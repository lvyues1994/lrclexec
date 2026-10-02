#include <atomic>
#include <chrono>
#include <cstdlib>
#include <example_interfaces/action/fibonacci.hpp>
#include <example_interfaces/srv/add_two_ints.hpp>
#include <future>
#include <iostream>
#include <lrclexec/ActionServer.h>
#include <lrclexec/ExecuteAction.h>
#include <lrclexec/Service.h>
#include <lrclexec/SpinWithScope.h>
#include <lrclexec/Topic.h>
#include <rclcpp/experimental/executors/events_executor/events_executor.hpp>
#include <rclcpp/rclcpp.hpp>
#include <rclcpp_action/create_client.hpp>
#include <rclcpp_lifecycle/lifecycle_node.hpp>
#include <rosgraph_msgs/msg/clock.hpp>
#include <std_msgs/msg/string.hpp>
#include <thread>

using namespace std::chrono_literals;
using Action = example_interfaces::action::Fibonacci;
using Service = example_interfaces::srv::AddTwoInts;
using Message = std_msgs::msg::String;
using EventsExecutor = rclcpp::experimental::executors::EventsExecutor;

namespace {
void check(bool condition, char const *message) {
    if (not condition)
        throw std::runtime_error{message};
}
[[noreturn]] void timedOut() {
    std::cerr << "node adapter operation timed out\n";
    std::_Exit(1); // Keep pending operations and borrowed receivers alive until process exit.
}
template <class Predicate> void waitUntil(Predicate predicate) {
    auto const deadline = std::chrono::steady_clock::now() + 3s;
    while (not predicate()) {
        if (std::chrono::steady_clock::now() >= deadline)
            timedOut();
        std::this_thread::sleep_for(1ms);
    }
}
template <class Value> Value await(std::future<Value> &future) {
    if (future.wait_for(3s) != std::future_status::ready)
        timedOut();
    return future.get();
}
struct IntegerReceiver {
    using receiver_concept = lexec::receiver_t;
    void set_value(int value = 1) && noexcept { promise->set_value(value); }
    void set_stopped() && noexcept { promise->set_value(0); }
    void set_error(std::exception_ptr error) && noexcept { promise->set_exception(error); }
    void set_error(lrclexec::ActionError<Action> error) && noexcept {
        promise->set_exception(std::make_exception_ptr(std::move(error)));
    }
    std::promise<int> *promise;
};
struct MissionFactory {
    auto operator()(std::shared_ptr<rclcpp_action::ServerGoalHandle<Action>> handle) const {
        ++*started;
        auto result = std::make_shared<Action::Result>();
        result->sequence = {handle->get_goal()->order};
        return lrclexec::schedule_after(scheduler, std::chrono::milliseconds{handle->get_goal()->order}) |
               lexec::then([result]() noexcept { return result; }) |
               lexec::let_stopped([scheduler = scheduler, cleanups = cleanups] {
                   auto cleanup =
                       lrclexec::schedule_after(scheduler, 10ms) | lexec::let_value([cleanups]() noexcept {
                           ++*cleanups;
                           return lexec::just_stopped();
                       });
                   return lexec::write_env(std::move(cleanup),
                                           lexec::prop{lexec::get_stop_token, lexec::never_stop_token{}});
               });
    }
    lrclexec::TimerScheduler scheduler;
    std::atomic<int> *started;
    std::atomic<int> *cleanups;
};
using AdaptedServer = decltype(lrclexec::make_action_server_preempt<Action>(
    std::declval<lrclexec::TimerScheduler>(), std::declval<lexec::counting_scope &>(), "",
    std::declval<MissionFactory>()));

template <class Node> struct Fixture {
    explicit Fixture(std::string const &mode)
        : node{std::make_shared<Node>("lrclexec_node_adapters")},
          peer{std::make_shared<rclcpp::Node>("lrclexec_node_adapters_peer")}, scheduler{node} {
        if (mode == "events")
            executor = std::make_unique<EventsExecutor>();
        else if (mode == "multi")
            executor =
                std::make_unique<rclcpp::executors::MultiThreadedExecutor>(rclcpp::ExecutorOptions{}, 4);
        else
            executor = std::make_unique<rclcpp::executors::SingleThreadedExecutor>();
        publisher = peer->create_publisher<Message>("node_topic", rclcpp::QoS{1});
        service = peer->create_service<Service>(
            "node_service", [](Service::Request::SharedPtr request, Service::Response::SharedPtr response) {
                response->sum = request->a + request->b;
            });
        serviceClient = node->template create_client<Service>("node_service");
        server.emplace(lrclexec::make_action_server_preempt<Action>(
            scheduler, scope, "node_action", MissionFactory{scheduler, &started, &cleanups}));
        client = rclcpp_action::create_client<Action>(node, "node_action");
        executor->add_node(node->get_node_base_interface());
        executor->add_node(peer);
        spinner = std::thread{[this] { executor->spin(); }};
        check(client->wait_for_action_server(3s), "node action discovery failed");
    }
    ~Fixture() {
        server->close();
        scope.close();
        scope.request_stop();
        (void)lexec::sync_wait(scope.join());
        executor->cancel();
        spinner.join(); // Release ROS Action resources only after executor collection has stopped.
    }
    void barrier() { (void)lexec::sync_wait(lexec::schedule(scheduler)); }

    std::shared_ptr<Node> node;
    rclcpp::Node::SharedPtr peer;
    lrclexec::TimerScheduler scheduler;
    lexec::counting_scope scope;
    std::atomic<int> started{0}, cleanups{0};
    std::unique_ptr<rclcpp::Executor> executor;
    rclcpp::Publisher<Message>::SharedPtr publisher;
    rclcpp::Service<Service>::SharedPtr service;
    rclcpp::Client<Service>::SharedPtr serviceClient;
    std::optional<AdaptedServer> server;
    rclcpp_action::Client<Action>::SharedPtr client;
    std::thread spinner;
};

template <class Node> void communicationTests(Fixture<Node> &fixture) {
    check(fixture.scheduler.nodeInterfaces().get_node_base_interface() ==
              fixture.node->get_node_base_interface(),
          "scheduler lost the original node interfaces");
    check(lexec::sync_wait(lrclexec::schedule_after(fixture.scheduler, 2ms)).has_value(),
          "node wall timer failed");
    Service::Request request;
    request.a = 19;
    request.b = 23;
    auto response =
        lexec::sync_wait(lrclexec::call_service(fixture.scheduler, fixture.serviceClient, request));
    check(response and std::get<0>(*response)->sum == 42, "node service failed");

    std::promise<int> promise;
    auto future = promise.get_future();
    auto message = lrclexec::wait_message<Message>(fixture.scheduler, "node_topic") |
                   lexec::then([](auto value) noexcept { return value->data == "node interfaces" ? 1 : -1; });
    auto operation = lexec::connect(std::move(message), IntegerReceiver{&promise});
    lexec::start(operation);
    waitUntil([&] { return fixture.publisher->get_subscription_count() == 1; });
    fixture.barrier();
    Message payload;
    payload.data = "node interfaces";
    fixture.publisher->publish(payload);
    check(await(future) == 1, "node topic failed");
    waitUntil([&] { return fixture.publisher->get_subscription_count() == 0; });

    Action::Goal goal;
    goal.order = 2;
    auto result = lexec::sync_wait(lrclexec::execute_action(fixture.scheduler, fixture.client, goal));
    check(result and std::get<0>(*result)->sequence == std::vector<int>{2}, "node action failed");
}

template <class Node> void cancellationTests(Fixture<Node> &fixture) {
    auto const previousStarts = fixture.started.load();
    auto const previousCleanups = fixture.cleanups.load();
    auto source = lexec::inplace_stop_source{};
    std::promise<int> promise;
    auto future = promise.get_future();
    Action::Goal goal;
    goal.order = 10000;
    auto work = lrclexec::execute_action(fixture.scheduler, fixture.client, goal) |
                lexec::then([](auto) noexcept { return 1; });
    auto operation = lexec::connect(
        lexec::write_env(std::move(work), lexec::prop{lexec::get_stop_token, source.get_token()}),
        IntegerReceiver{&promise});
    lexec::start(operation);
    waitUntil([&] { return fixture.started.load() > previousStarts; });
    source.request_stop();
    check(await(future) == 0 and fixture.cleanups.load() == previousCleanups + 1,
          "node action cancellation skipped asynchronous cleanup");
    auto topic = lrclexec::wait_message<Message>(fixture.scheduler, "node_topic") |
                 lexec::then([](auto) noexcept { return false; });
    auto timeout =
        lrclexec::schedule_after(fixture.scheduler, 10ms) | lexec::then([]() noexcept { return true; });
    auto result = lexec::sync_wait(lexec::when_any(std::move(topic), std::move(timeout)));
    check(result and std::get<0>(*result), "node topic timeout failed");
}

void lifetimeTests() {
    auto node = std::make_shared<rclcpp_lifecycle::LifecycleNode>("lrclexec_lifecycle_owner");
    auto weak = std::weak_ptr<rclcpp_lifecycle::LifecycleNode>{node};
    {
        auto scheduler = lrclexec::TimerScheduler{node};
        node.reset();
        check(not weak.expired(), "scheduler did not retain the complete lifecycle node");
        auto rejected = false;
        try {
            (void)scheduler.node();
        } catch (std::logic_error const &) {
            rejected = true;
        }
        check(rejected, "lifecycle scheduler fabricated an ordinary node");
    }
    check(weak.expired(), "scheduler leaked the lifecycle node");
    auto rejected = false;
    try {
        auto scheduler = lrclexec::TimerScheduler{std::shared_ptr<rclcpp_lifecycle::LifecycleNode>{}};
    } catch (std::invalid_argument const &) {
        rejected = true;
    }
    check(rejected, "null lifecycle node was accepted");
    struct DerivedNode final : rclcpp::Node {
        DerivedNode() : Node{"lrclexec_derived_node"} {}
    };
    auto derived = std::make_shared<DerivedNode>();
    auto scheduler = lrclexec::TimerScheduler{derived};
    check(&scheduler.node() == derived.get(), "ordinary Node subclass lost its accessor");
}

void nodeClockTest(Fixture<rclcpp_lifecycle::LifecycleNode> &fixture) {
    auto options = rclcpp::NodeOptions{}.parameter_overrides({rclcpp::Parameter{"use_sim_time", true}});
    auto node = std::make_shared<rclcpp_lifecycle::LifecycleNode>("lrclexec_lifecycle_clock", options);
    auto scheduler = lrclexec::TimerScheduler{node, lrclexec::TimerClock::node};
    fixture.executor->add_node(node->get_node_base_interface());
    auto publisher = fixture.peer->create_publisher<rosgraph_msgs::msg::Clock>("/clock", rclcpp::ClockQoS{});
    waitUntil([&] { return publisher->get_subscription_count() > 0; });
    auto setClock = [&](int seconds) {
        rosgraph_msgs::msg::Clock value;
        value.clock.sec = seconds;
        publisher->publish(value);
        waitUntil([&] { return node->now().seconds() == seconds; });
    };
    setClock(10);
    for (bool cancel : {false, true}) {
        std::promise<int> promise;
        auto future = promise.get_future();
        auto stop = lexec::inplace_stop_source{};
        auto operation =
            lexec::connect(lexec::write_env(lrclexec::schedule_after(scheduler, 1s),
                                            lexec::prop{lexec::get_stop_token, stop.get_token()}),
                           IntegerReceiver{&promise});
        lexec::start(operation);
        (void)lexec::sync_wait(lexec::schedule(scheduler));
        auto const paused = future.wait_for(20ms) == std::future_status::timeout;
        if (cancel)
            stop.request_stop();
        else
            setClock(12);
        check(await(future) == (cancel ? 0 : 1) and paused, "lifecycle node clock/cancellation failed");
    }
    fixture.executor->remove_node(node->get_node_base_interface());
}

void eventsQueueTests() {
    auto node = std::make_shared<rclcpp::Node>("lrclexec_events_queue");
    auto scheduler = lrclexec::TimerScheduler{node};
    auto executor = EventsExecutor{};
    auto calls = 0;
    scheduler.executionContext()->post([&] { ++calls; }); // Before listener registration.
    executor.add_node(node);
    auto pump = [&](auto predicate) {
        auto const deadline = std::chrono::steady_clock::now() + 3s;
        while (not predicate()) {
            if (std::chrono::steady_clock::now() >= deadline)
                timedOut();
            executor.spin_once(10ms);
        }
    };
    pump([&] { return calls == 1; });
    executor.remove_node(node);
    for (int i = 0; i < 3; ++i)
        executor.spin_once(1ms);
    check(calls == 1, "node removal executed unposted work");
    std::shared_ptr<rclcpp::Waitable> queue;
    scheduler.callbackGroup()->collect_all_ptrs([](auto) {}, [](auto) {}, [](auto) {}, [](auto) {},
                                                [&](auto waitable) { queue = std::move(waitable); });
    check(static_cast<bool>(queue), "queue waitable disappeared after node removal");
    auto consumed = 0;
    // Simulate a listener consuming a notification without executing its task.
    // Registration must notice pending work even when guard unread_count is zero.
    queue->set_on_ready_callback([&](std::size_t, int) { ++consumed; });
    auto const before = consumed;
    scheduler.executionContext()->post([&] { ++calls; });
    check(consumed == before + 1 and calls == 1, "readiness listener executed user work");
    queue->clear_on_ready_callback();
    executor.add_node(node);
    pump([&] { return calls == 2; });

    auto observed = false;
    scheduler.executionContext()->post([] { throw std::runtime_error{"queue failure"}; });
    scheduler.executionContext()->post([&] { ++calls; });
    try {
        pump([&] { return calls == 3; });
    } catch (std::runtime_error const &) {
        observed = true;
    }
    check(observed and calls == 3, "EventsExecutor abandoned a batch after an exception");
    scheduler.executionContext()->post([&] { ++calls; });
    pump([&] { return calls == 4; });

    auto scope = lexec::counting_scope{};
    auto stop = lexec::inplace_stop_source{};
    auto cleaned = false;
    auto failed = false;
    auto work = lrclexec::schedule_after(scheduler, 10s) | lexec::let_stopped([&] {
                    auto cleanup = lrclexec::schedule_after(scheduler, 10ms) |
                                   lexec::then([&]() noexcept { cleaned = true; });
                    return lexec::write_env(std::move(cleanup),
                                            lexec::prop{lexec::get_stop_token, lexec::never_stop_token{}});
                });
    lexec::spawn(std::move(work) | lexec::upon_error([&](std::exception_ptr) noexcept { failed = true; }),
                 scope.get_token());
    scheduler.executionContext()->post([&] { stop.request_stop(); });
    lrclexec::spin_with_scope(executor, scope, stop.get_token());
    check(cleaned and not failed and node->get_node_base_interface()->get_context()->is_valid(),
          "EventsExecutor scope did not drain asynchronous cleanup");
}
} // namespace

int main(int argc, char **argv) {
    rclcpp::init(argc, argv, rclcpp::InitOptions{}, rclcpp::SignalHandlerOptions::None);
    try {
        auto const mode = argc > 1 ? std::string{argv[1]} : "single";
        if (mode == "events-node") {
            eventsQueueTests();
            Fixture<rclcpp::Node> fixture{"events"};
            communicationTests(fixture);
            cancellationTests(fixture);
        } else {
            lifetimeTests();
            Fixture<rclcpp_lifecycle::LifecycleNode> fixture{mode};
            communicationTests(fixture); // Unconfigured nodes may still perform ordinary work.
            check(fixture.node->configure().label() == "inactive", "configure failed");
            communicationTests(fixture);
            check(fixture.node->activate().label() == "active", "activate failed");
            communicationTests(fixture);
            check(fixture.node->deactivate().label() == "inactive", "deactivate failed");
            communicationTests(fixture); // Inactive does not automatically stop these adapters.
            cancellationTests(fixture);
            if (mode != "events")
                nodeClockTest(fixture);
        }
        rclcpp::shutdown();
        std::cout << "Node interfaces, lifecycle states and executor adapter checks passed\n";
        return 0;
    } catch (std::exception const &error) {
        std::cerr << error.what() << '\n';
        rclcpp::shutdown();
        return 1;
    }
}
