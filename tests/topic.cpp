#include <atomic>
#include <chrono>
#include <cstdlib>
#include <future>
#include <iostream>
#include <lrclexec/Topic.h>
#include <rclcpp/rclcpp.hpp>
#include <std_msgs/msg/string.hpp>
#include <thread>

using namespace std::chrono_literals;
using Message = std_msgs::msg::String;
using MessagePtr = std::shared_ptr<Message const>;

namespace {
void check(bool condition, char const *message) {
    if (not condition)
        throw std::runtime_error{message};
}
[[noreturn]] void timedOut(char const *message) {
    std::cerr << message << '\n';
    std::_Exit(1); // Do not destroy an operation that has not completed.
}
template <class Predicate> void waitUntil(Predicate predicate) {
    auto const deadline = std::chrono::steady_clock::now() + 3s;
    while (not predicate()) {
        if (std::chrono::steady_clock::now() >= deadline)
            timedOut("topic event timed out");
        std::this_thread::sleep_for(1ms);
    }
}
template <class Value> Value await(std::future<Value> &future) {
    if (future.wait_for(3s) != std::future_status::ready)
        timedOut("topic sender did not complete");
    return future.get();
}
struct Observation {
    std::promise<MessagePtr> result;
    std::atomic<int> calls{0};
};
struct Receiver {
    using receiver_concept = lexec::receiver_t;
    void set_value(MessagePtr message) && noexcept {
        ++observation->calls;
        observation->result.set_value(std::move(message));
    }
    void set_stopped() && noexcept {
        ++observation->calls;
        observation->result.set_value({});
    }
    void set_error(std::exception_ptr error) && noexcept {
        ++observation->calls;
        observation->result.set_exception(error);
    }
    Observation *observation;
};

struct Fixture {
    Fixture(bool multi, bool ipc)
        : node{std::make_shared<rclcpp::Node>("lrclexec_topic_client",
                                              rclcpp::NodeOptions{}.use_intra_process_comms(ipc))},
          peer{std::make_shared<rclcpp::Node>("lrclexec_topic_peer",
                                              rclcpp::NodeOptions{}.use_intra_process_comms(ipc))},
          scheduler{node} {
        if (multi)
            executor =
                std::make_unique<rclcpp::executors::MultiThreadedExecutor>(rclcpp::ExecutorOptions{}, 4);
        else
            executor = std::make_unique<rclcpp::executors::SingleThreadedExecutor>();
        executor->add_node(node);
        executor->add_node(peer);
        publisher = peer->create_publisher<Message>("lrclexec_topic", rclcpp::QoS{32});
        spinner = std::thread{[this] { executor->spin(); }};
    }
    ~Fixture() {
        executor->cancel();
        spinner.join();
    }
    auto wait() { return lrclexec::wait_message<Message>(scheduler, "lrclexec_topic"); }
    void publish(std::string value) {
        auto message = std::make_unique<Message>();
        message->data = std::move(value);
        publisher->publish(std::move(message));
    }
    void subscribed(std::size_t count) {
        auto const deadline = std::chrono::steady_clock::now() + 3s;
        while (publisher->get_subscription_count() != count) {
            if (std::chrono::steady_clock::now() >= deadline) {
                std::cerr << "subscriptions: expected=" << count
                          << " actual=" << publisher->get_subscription_count() << '\n';
                timedOut("topic subscription graph did not converge");
            }
            std::this_thread::sleep_for(1ms);
        }
        if (count > 0)
            barrier(); // DDS discovery can precede IPC and callback-group setup.
    }
    void barrier() { (void)lexec::sync_wait(lexec::schedule(scheduler)); }

    rclcpp::Node::SharedPtr node;
    rclcpp::Node::SharedPtr peer;
    lrclexec::TimerScheduler scheduler;
    std::unique_ptr<rclcpp::Executor> executor;
    rclcpp::Publisher<Message>::SharedPtr publisher;
    std::thread spinner;
};

void basicTests(Fixture &fixture) {
    // Retain the whole burst: depth=1 may overwrite messages before delivery.
    auto sender = lrclexec::wait_message<Message>(fixture.scheduler, "lrclexec_topic", rclcpp::QoS{32});
    fixture.subscribed(0); // Constructing a sender does not subscribe.
    MessagePtr retained;
    for (int round = 0; round < 2; ++round) {
        auto observation = Observation{};
        auto future = observation.result.get_future();
        auto operation = lexec::connect(sender, Receiver{&observation});
        lexec::start(operation);
        fixture.subscribed(1);
        auto const first = std::to_string(round);
        fixture.publish(first);
        for (int i = 0; i < 20; ++i)
            fixture.publish("later");
        retained = await(future);
        check(retained and retained->data == first, "first delivered topic message was lost");
        fixture.subscribed(0);
        fixture.barrier();
        check(observation.calls == 1, "topic sender completed more than once");
    }
    check(retained->data == "1", "message did not outlive subscription");

    auto first = Observation{};
    auto second = Observation{};
    auto a = first.result.get_future();
    auto b = second.result.get_future();
    auto opA = lexec::connect(sender, Receiver{&first});
    auto opB = lexec::connect(sender, Receiver{&second});
    lexec::start(opA);
    lexec::start(opB);
    fixture.subscribed(2);
    fixture.publish("independent");
    check(await(a)->data == "independent" and await(b)->data == "independent",
          "independent topic waits interfered");
    fixture.subscribed(0);
}

void stopTests(Fixture &fixture) {
    for (bool preStopped : {true, false}) {
        auto source = lexec::inplace_stop_source{};
        if (preStopped)
            source.request_stop();
        auto observation = Observation{};
        auto future = observation.result.get_future();
        auto operation = lexec::connect(
            lexec::write_env(fixture.wait(), lexec::prop{lexec::get_stop_token, source.get_token()}),
            Receiver{&observation});
        lexec::start(operation);
        if (not preStopped) {
            fixture.subscribed(1);
            source.request_stop();
        }
        check(not await(future), "stopped topic wait produced a message");
        fixture.subscribed(0);
        check(observation.calls == 1, "topic stop completed twice");
    }
    auto wait = fixture.wait() | lexec::then([](auto) noexcept { return false; });
    auto timeout =
        lrclexec::schedule_after(fixture.scheduler, 20ms) | lexec::then([]() noexcept { return true; });
    auto result = lexec::sync_wait(lexec::when_any(std::move(wait), std::move(timeout)));
    check(result and std::get<0>(*result), "topic timeout did not win");
    fixture.subscribed(0);

    for (int i = 0; i < 50; ++i) {
        auto source = lexec::inplace_stop_source{};
        auto observation = Observation{};
        auto future = observation.result.get_future();
        auto operation = lexec::connect(
            lexec::write_env(fixture.wait(), lexec::prop{lexec::get_stop_token, source.get_token()}),
            Receiver{&observation});
        lexec::start(operation);
        fixture.subscribed(1);
        auto cancel = std::thread{[&] { source.request_stop(); }};
        fixture.publish("race");
        cancel.join();
        auto message = await(future);
        check(not message or message->data == "race", "topic race corrupted message");
        fixture.subscribed(0);
        fixture.barrier();
        check(observation.calls == 1, "topic message/stop race completed twice");
    }
}

struct ControlledContext final : lrclexec::detail::ExecutionContext {
    explicit ControlledContext(std::shared_ptr<lrclexec::detail::ExecutionContext> inner_)
        : inner{std::move(inner_)} {}
    void post(std::function<void()> task) override {
        if (failNext.exchange(false))
            throw std::runtime_error{"injected topic post failure"};
        if (holdNext.exchange(false)) {
            auto const lock = std::lock_guard<std::mutex>{mutex};
            held = std::move(task);
            captured.store(true);
        } else
            inner->post(std::move(task));
    }
    void release() {
        std::function<void()> task;
        {
            auto const lock = std::lock_guard<std::mutex>{mutex};
            task = std::exchange(held, {});
        }
        if (task)
            inner->post(std::move(task));
    }
    rclcpp::Node &node() const noexcept override { return inner->node(); }
    rclcpp::CallbackGroup::SharedPtr callbackGroup() const noexcept override {
        return inner->callbackGroup();
    }
    std::shared_ptr<lrclexec::detail::ExecutionContext> inner;
    std::atomic<bool> failNext{false}, holdNext{false}, captured{false};
    std::mutex mutex;
    std::function<void()> held;
};

void queuedMessageTest(Fixture &fixture) {
    auto context = std::make_shared<ControlledContext>(fixture.scheduler.executionContext());
    auto scheduler = lrclexec::TimerScheduler{context};
    auto observation = Observation{};
    auto future = observation.result.get_future();
    {
        auto source = lexec::inplace_stop_source{};
        auto operation =
            lexec::connect(lexec::write_env(lrclexec::wait_message<Message>(scheduler, "lrclexec_topic"),
                                            lexec::prop{lexec::get_stop_token, source.get_token()}),
                           Receiver{&observation});
        lexec::start(operation);
        fixture.subscribed(1);
        context->holdNext.store(true);
        fixture.publish("queued");
        waitUntil([&] { return context->captured.load(); });
        source.request_stop();
        check(not await(future), "queued topic message beat stop");
        fixture.subscribed(0);
    } // Destroy operation and source before running the stale queued message.
    context->release();
    fixture.barrier();
    check(observation.calls == 1, "stale topic delivery completed twice");
}

void faultTests(Fixture &fixture) {
    for (int phase = 0; phase < 3; ++phase) {
        auto context = std::make_shared<ControlledContext>(fixture.scheduler.executionContext());
        auto scheduler = lrclexec::TimerScheduler{context};
        auto source = lexec::inplace_stop_source{};
        auto observation = Observation{};
        auto future = observation.result.get_future();
        auto operation =
            lexec::connect(lexec::write_env(lrclexec::wait_message<Message>(scheduler, "lrclexec_topic"),
                                            lexec::prop{lexec::get_stop_token, source.get_token()}),
                           Receiver{&observation});
        if (phase == 0)
            context->failNext.store(true);
        lexec::start(operation);
        if (phase != 0) {
            fixture.subscribed(1);
            context->failNext.store(true);
            if (phase == 1)
                fixture.publish("failure");
            else
                source.request_stop();
        }
        auto failed = false;
        try {
            (void)await(future);
        } catch (std::runtime_error const &) {
            failed = true;
        }
        check(failed and observation.calls == 1, "topic post failure was lost");
        fixture.subscribed(0);
    }
    // Stop registration calls synchronously for an already stopped source.
    // Its failed post must not reset the callback while it is being emplaced.
    {
        auto context = std::make_shared<ControlledContext>(fixture.scheduler.executionContext());
        auto scheduler = lrclexec::TimerScheduler{context};
        auto source = lexec::inplace_stop_source{};
        source.request_stop();
        auto observation = Observation{};
        auto future = observation.result.get_future();
        auto operation =
            lexec::connect(lexec::write_env(lrclexec::wait_message<Message>(scheduler, "lrclexec_topic"),
                                            lexec::prop{lexec::get_stop_token, source.get_token()}),
                           Receiver{&observation});
        context->holdNext.store(true);
        lexec::start(operation);
        context->failNext.store(true);
        context->release();
        auto failed = false;
        try {
            (void)await(future);
        } catch (std::runtime_error const &) {
            failed = true;
        }
        check(failed and observation.calls == 1, "topic stop registration post failure was lost");
        fixture.subscribed(0);
    }
    auto observation = Observation{};
    auto future = observation.result.get_future();
    auto operation = lexec::connect(lrclexec::wait_message<Message>(fixture.scheduler, "invalid topic!"),
                                    Receiver{&observation});
    lexec::start(operation);
    auto failed = false;
    try {
        (void)await(future);
    } catch (rclcpp::exceptions::InvalidTopicNameError const &) {
        failed = true;
    }
    check(failed and observation.calls == 1, "subscription creation error was lost");
    fixture.subscribed(0);
}

void qosTests(Fixture &fixture) {
    auto qos = rclcpp::QoS{1}.transient_local();
    auto publisher = fixture.peer->create_publisher<Message>("lrclexec_retained_topic", qos);
    auto message = Message{};
    message.data = "retained before subscription";
    publisher->publish(message);
    auto result =
        lexec::sync_wait(lrclexec::wait_message<Message>(fixture.scheduler, "lrclexec_retained_topic", qos));
    check(result and std::get<0>(*result)->data == message.data, "transient-local QoS was lost");

    auto sensor = fixture.peer->create_publisher<Message>("lrclexec_sensor_topic", rclcpp::SensorDataQoS{});
    auto observation = Observation{};
    auto future = observation.result.get_future();
    auto operation = lexec::connect(
        lrclexec::wait_message<Message>(fixture.scheduler, "lrclexec_sensor_topic", rclcpp::SensorDataQoS{}),
        Receiver{&observation});
    lexec::start(operation);
    waitUntil([&] { return sensor->get_subscription_count() > 0; });
    fixture.barrier();
    message.data = "best effort";
    sensor->publish(message);
    check(await(future)->data == message.data, "sensor QoS was lost");
}

void loanOwnershipTest(Fixture &fixture) {
    auto context = std::make_shared<ControlledContext>(fixture.scheduler.executionContext());
    auto scheduler = lrclexec::TimerScheduler{context};
    auto observation = Observation{};
    auto future = observation.result.get_future();
    auto operation =
        lexec::connect(lrclexec::wait_message<Message>(scheduler, "lrclexec_topic"), Receiver{&observation});
    lexec::start(operation);
    fixture.subscribed(1);
    std::shared_ptr<rclcpp::Subscription<Message>> subscription;
    context->callbackGroup()->collect_all_ptrs(
        [&](auto base) {
            if (std::string{base->get_topic_name()} == fixture.publisher->get_topic_name())
                subscription = std::dynamic_pointer_cast<rclcpp::Subscription<Message>>(base);
        },
        [](auto) {}, [](auto) {}, [](auto) {}, [](auto) {});
    check(static_cast<bool>(subscription), "topic subscription was not registered");
    context->holdNext.store(true);
    auto metadata = rmw_message_info_t{};
    metadata.publisher_gid = fixture.publisher->get_gid();
    metadata.publisher_gid.data[0] ^= 0xff; // A valid GID from a non-IPC publisher.
    auto info = rclcpp::MessageInfo{metadata};
    {
        auto loan = Message{};
        loan.data = "owned before loan return";
        subscription->handle_loaned_message(&loan, info);
        check(context->captured.load(), "loan callback was not queued");
        loan.data = "reused loan buffer";
        for (int i = 0; i < 10; ++i)
            subscription->handle_loaned_message(&loan, info);
    }
    // The test's SDK owner must go away before sender completion unregisters it.
    subscription.reset();
    context->release();
    auto result = await(future);
    fixture.subscribed(0);
    check(result->data == "owned before loan return", "sender retained borrowed DDS loan memory");
}
} // namespace

int main(int argc, char **argv) {
    auto const multi = argc > 1 and std::string{argv[1]} == "multi";
    rclcpp::init(argc, argv, rclcpp::InitOptions{}, rclcpp::SignalHandlerOptions::None);
    try {
        for (bool ipc : {false, true}) {
            Fixture fixture{multi, ipc};
            std::cerr << "basic checks ipc=" << ipc << '\n';
            basicTests(fixture);
            std::cerr << "stop checks\n";
            stopTests(fixture);
            std::cerr << "queued message checks\n";
            queuedMessageTest(fixture);
            std::cerr << "fault checks\n";
            faultTests(fixture);
            std::cerr << "QoS checks\n";
            qosTests(fixture);
            std::cerr << "loan checks\n";
            loanOwnershipTest(fixture);
        }
        rclcpp::shutdown();
        std::cout << "Topic cancellation, QoS, IPC, loan ownership and fault checks passed\n";
        return 0;
    } catch (std::exception const &error) {
        std::cerr << error.what() << '\n';
        rclcpp::shutdown();
        return 1;
    }
}
