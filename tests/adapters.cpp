#include <atomic>
#include <chrono>
#include <cstdlib>
#include <example_interfaces/srv/add_two_ints.hpp>
#include <future>
#include <iostream>
#include <lrclexec/Service.h>
#include <mutex>
#include <rclcpp/rclcpp.hpp>
#include <rosgraph_msgs/msg/clock.hpp>
#include <thread>
#include <vector>

using namespace std::chrono_literals;
using Service = example_interfaces::srv::AddTwoInts;

namespace {
void check(bool condition, char const *message) {
    if (not condition)
        throw std::runtime_error{message};
}
template <class Predicate> void waitUntil(Predicate predicate) {
    auto deadline = std::chrono::steady_clock::now() + 3s;
    while (not predicate()) {
        check(std::chrono::steady_clock::now() < deadline, "adapter event timed out");
        std::this_thread::sleep_for(1ms);
    }
}
struct Fixture {
    Fixture() : scheduler{node} {
        executor.add_node(node);
        executor.add_node(peer);
        server = peer->create_service<Service>(
            "lrclexec_adapter_service",
            [this](std::shared_ptr<rmw_request_id_t> header, Service::Request::SharedPtr request) {
                if (hold.load()) {
                    auto lock = std::lock_guard<std::mutex>{mutex};
                    pending.push_back({std::move(header), std::move(request)});
                    ++requests;
                } else {
                    Service::Response response;
                    response.sum = request->a + request->b;
                    ++requests;
                    server->send_response(*header, response);
                }
            });
        client = node->create_client<Service>("lrclexec_adapter_service");
        spinner = std::thread{[this] { executor.spin(); }};
        check(client->wait_for_service(3s), "service discovery failed");
    }
    ~Fixture() {
        executor.cancel();
        spinner.join();
    }
    auto call(rclcpp::Client<Service>::SharedPtr target = {}) {
        Service::Request request;
        request.a = 19;
        request.b = 23;
        return lrclexec::call_service(scheduler, target ? target : client, request);
    }
    void release() {
        std::vector<Pending> replies;
        {
            auto lock = std::lock_guard<std::mutex>{mutex};
            replies.swap(pending);
        }
        for (auto &reply : replies) {
            Service::Response response;
            response.sum = reply.request->a + reply.request->b;
            server->send_response(*reply.header, response);
        }
    }
    struct Pending {
        std::shared_ptr<rmw_request_id_t> header;
        Service::Request::SharedPtr request;
    };
    rclcpp::Node::SharedPtr node = std::make_shared<rclcpp::Node>("lrclexec_adapters");
    rclcpp::Node::SharedPtr peer = std::make_shared<rclcpp::Node>("lrclexec_adapters_peer");
    lrclexec::TimerScheduler scheduler;
    rclcpp::executors::MultiThreadedExecutor executor{rclcpp::ExecutorOptions{}, 4};
    rclcpp::Service<Service>::SharedPtr server;
    rclcpp::Client<Service>::SharedPtr client;
    std::thread spinner;
    std::atomic<int> requests{0};
    std::atomic<bool> hold{false};
    std::mutex mutex;
    std::vector<Pending> pending;
};

struct IntegerReceiver {
    using receiver_concept = lexec::receiver_t;
    void set_value(int value = 1) && noexcept { promise->set_value(value); }
    void set_stopped() && noexcept { promise->set_value(0); }
    void set_error(std::exception_ptr error) && noexcept { promise->set_exception(error); }
    std::promise<int> *promise;
};

void serviceTests(Fixture &fixture) {
    for (int i = 0; i < 100; ++i) {
        auto response = lexec::sync_wait(fixture.call());
        check(response and std::get<0>(*response)->sum == 42, "service response lost");
    }
    auto source = lexec::inplace_stop_source{};
    source.request_stop();
    auto before = fixture.requests.load();
    check(not lexec::sync_wait(
              lexec::write_env(fixture.call(), lexec::prop{lexec::get_stop_token, source.get_token()})),
          "pre-stopped service ran");
    check(fixture.requests.load() == before, "pre-stopped service sent request");

    fixture.hold.store(true);
    auto cancellation = lexec::inplace_stop_source{};
    auto future = std::async(std::launch::async, [&] {
        return lexec::sync_wait(
            lexec::write_env(fixture.call(), lexec::prop{lexec::get_stop_token, cancellation.get_token()}));
    });
    waitUntil([&] { return fixture.requests.load() > before; });
    cancellation.request_stop();
    auto const stoppedLocally = future.wait_for(3s) == std::future_status::ready;
    if (not stoppedLocally) {
        fixture.release();
        if (future.wait_for(3s) != std::future_status::ready) {
            std::cerr << "service cancellation could not be rescued\n";
            std::_Exit(1);
        }
    }
    check(not future.get(), "service stop produced value");
    check(stoppedLocally, "service stop waited for remote response");
    check(fixture.client->prune_pending_requests() == 0, "service stop leaked pending request");
    fixture.release(); // A late remote reply must be harmless.

    auto timed = fixture.call() | lexec::then([](auto) noexcept { return false; });
    auto timeout =
        lrclexec::schedule_after(fixture.scheduler, 20ms) | lexec::then([]() noexcept { return true; });
    auto result = lexec::sync_wait(lexec::when_any(std::move(timed), std::move(timeout)));
    check(result and std::get<0>(*result), "service timeout did not win");
    check(fixture.client->prune_pending_requests() == 0, "service timeout leaked pending request");
    fixture.release();
    fixture.hold.store(false);
    auto response = lexec::sync_wait(fixture.call());
    check(response and std::get<0>(*response)->sum == 42, "late reply affected next request");

    auto missing = fixture.node->create_client<Service>("lrclexec_missing_service");
    auto unavailable = fixture.call(missing) | lexec::then([](auto) noexcept { return false; });
    auto deadline =
        lrclexec::schedule_after(fixture.scheduler, 40ms) | lexec::then([]() noexcept { return true; });
    result = lexec::sync_wait(lexec::when_any(std::move(unavailable), std::move(deadline)));
    check(result and std::get<0>(*result), "service discovery was not cancellable");
    check(missing->prune_pending_requests() == 0, "discovery sent a request before service existed");

    auto lateClient = fixture.node->create_client<Service>("lrclexec_late_service");
    std::promise<int> latePromise;
    auto late = latePromise.get_future();
    auto lateOperation = lexec::connect(fixture.call(lateClient) | lexec::then([](auto reply) noexcept {
                                            return static_cast<int>(reply->sum);
                                        }),
                                        IntegerReceiver{&latePromise});
    lexec::start(lateOperation);
    (void)lexec::sync_wait(lexec::schedule(fixture.scheduler));
    auto const initiallyPending = late.wait_for(50ms) == std::future_status::timeout;
    auto lateRequests = std::atomic<int>{0};
    auto lateServer = fixture.peer->create_service<Service>(
        "lrclexec_late_service",
        [&](Service::Request::SharedPtr request, Service::Response::SharedPtr reply) {
            ++lateRequests;
            reply->sum = request->a + request->b;
        });
    if (late.wait_for(3s) != std::future_status::ready) {
        std::cerr << "service discovery did not recover after server appeared\n";
        std::_Exit(1);
    }
    auto lateSum = late.get();
    check(initiallyPending and lateSum == 42 and lateRequests.load() == 1,
          "late service discovery did not send exactly once");
    check(lateClient->prune_pending_requests() == 0, "late discovery leaked a request");
}

struct HoldingContext final : lrclexec::detail::ExecutionContext {
    explicit HoldingContext(std::shared_ptr<lrclexec::detail::ExecutionContext> inner_)
        : inner{std::move(inner_)} {}
    void post(std::function<void()> task) override {
        if (holdNext.exchange(false)) {
            auto lock = std::lock_guard<std::mutex>{mutex};
            held = std::move(task);
            captured.store(true);
        } else
            inner->post(std::move(task));
    }
    void release() {
        std::function<void()> task;
        {
            auto lock = std::lock_guard<std::mutex>{mutex};
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
    std::atomic<bool> holdNext{false}, captured{false};
    std::mutex mutex;
    std::function<void()> held;
};

void queuedResponseAfterStopTest(Fixture &fixture) {
    auto context = std::make_shared<HoldingContext>(fixture.scheduler.executionContext());
    auto scheduler = lrclexec::TimerScheduler{context};
    auto before = fixture.requests.load();
    fixture.hold.store(true);
    std::promise<int> promise;
    auto future = promise.get_future();
    auto stopped = false;
    {
        auto source = lexec::inplace_stop_source{};
        auto request = Service::Request{};
        auto operation =
            lexec::connect(lexec::write_env(lrclexec::call_service(scheduler, fixture.client, request) |
                                                lexec::then([](auto) noexcept {}),
                                            lexec::prop{lexec::get_stop_token, source.get_token()}),
                           IntegerReceiver{&promise});
        lexec::start(operation);
        waitUntil([&] { return fixture.requests.load() > before; });
        context->holdNext.store(true);
        fixture.release();
        waitUntil([&] { return context->captured.load(); });
        // SDK already took the response, so remove_pending_request returns false.
        source.request_stop();
        if (future.wait_for(3s) != std::future_status::ready) {
            context->release();
            if (future.wait_for(3s) != std::future_status::ready) {
                std::cerr << "queued service response could not be rescued\n";
                std::_Exit(1);
            }
        }
        stopped = future.get() == 0;
    } // Destroy operation and borrowed stop source before the queued response runs.
    context->release();
    (void)lexec::sync_wait(lexec::schedule(fixture.scheduler));
    fixture.hold.store(false);
    check(stopped, "queued response beat local stop");
    check(fixture.client->prune_pending_requests() == 0, "queued response left pending request");
}

void rosTimeTests(Fixture &fixture) {
    auto options = rclcpp::NodeOptions{}.parameter_overrides({rclcpp::Parameter{"use_sim_time", true}});
    auto node = std::make_shared<rclcpp::Node>("lrclexec_clock_test", options);
    fixture.executor.add_node(node);
    auto scheduler = lrclexec::TimerScheduler{node, lrclexec::TimerClock::node};
    auto publisher = fixture.peer->create_publisher<rosgraph_msgs::msg::Clock>("/clock", rclcpp::ClockQoS{});
    waitUntil([&] { return publisher->get_subscription_count() > 0; });
    auto setClock = [&](int seconds) {
        rosgraph_msgs::msg::Clock clock;
        clock.clock.sec = seconds;
        publisher->publish(clock);
        waitUntil([&] { return node->now().nanoseconds() == std::int64_t{seconds} * 1000000000; });
    };
    setClock(10);
    auto attributes = lexec::get_env(lrclexec::schedule_after(scheduler, 1s));
    check(lexec::get_completion_scheduler<lexec::set_value_t>(attributes) == scheduler,
          "timer attributes lost node clock");
    for (int scenario = 0; scenario < 3; ++scenario) {
        setClock(10);
        std::promise<int> promise;
        auto future = promise.get_future();
        auto source = lexec::inplace_stop_source{};
        auto operation =
            lexec::connect(lexec::write_env(lrclexec::schedule_after(scheduler, 1s),
                                            lexec::prop{lexec::get_stop_token, source.get_token()}),
                           IntegerReceiver{&promise});
        lexec::start(operation);
        (void)lexec::sync_wait(lexec::schedule(scheduler));
        auto paused = future.wait_for(40ms) == std::future_status::timeout;
        auto resetWaited = true;
        if (scenario == 0)
            setClock(12); // Forward jump passes the deadline.
        else if (scenario == 1) {
            setClock(5); // Backward jump before last_call_time resets the ROS timer.
            resetWaited = future.wait_for(20ms) == std::future_status::timeout;
            setClock(6);
        } else
            source.request_stop(); // Stop must work even while ROS time is paused.
        auto ready = future.wait_for(3s) == std::future_status::ready;
        if (not ready) {
            source.request_stop();
            if (future.wait_for(3s) != std::future_status::ready) {
                std::cerr << "paused ROS timer cancellation could not be rescued\n";
                std::_Exit(1);
            }
        }
        auto outcome = future.get();
        check(paused and resetWaited and ready, "ROS timer pause/jump behavior failed");
        check(outcome == (scenario == 2 ? 0 : 1), "ROS timer completion kind was wrong");
    }
    fixture.executor.remove_node(node);
}
} // namespace

int main(int argc, char **argv) {
    rclcpp::init(argc, argv, rclcpp::InitOptions{}, rclcpp::SignalHandlerOptions::None);
    try {
        {
            Fixture fixture;
            serviceTests(fixture);
            queuedResponseAfterStopTest(fixture);
            rosTimeTests(fixture);
        }
        rclcpp::shutdown();
        std::cout << "Service and ROS time adapters passed\n";
        return 0;
    } catch (std::exception const &error) {
        std::cerr << error.what() << '\n';
        rclcpp::shutdown();
        return 1;
    }
}
