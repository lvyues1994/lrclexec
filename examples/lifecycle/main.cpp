#include <algorithm>
#include <atomic>
#include <chrono>
#include <cstdlib>
#include <future>
#include <iostream>
#include <lrclexec/TimerScheduler.h>
#include <rclcpp/experimental/executors/events_executor/events_executor.hpp>
#include <rclcpp/rclcpp.hpp>
#include <rclcpp_lifecycle/lifecycle_node.hpp>
#include <thread>

using namespace std::chrono_literals;
using CallbackReturn = rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn;
using Node = rclcpp_lifecycle::LifecycleNode;

namespace {
void check(bool const condition, char const *const message) {
    if (not condition)
        throw std::runtime_error{message};
}
struct Statistics {
    int starts = 0, cleanups = 0, errors = 0, live = 0, peak = 0;
};
struct Resource {
    explicit Resource(Statistics &statistics_) : statistics{statistics_} {
        ++statistics.live;
        statistics.peak = std::max(statistics.peak, statistics.live);
    }
    ~Resource() { --statistics.live; }
    Resource(Resource const &) = delete;
    Resource &operator=(Resource const &) = delete;
    Resource(Resource &&) = delete;
    Resource &operator=(Resource &&) = delete;
    Statistics &statistics;
};
struct Joined {
    using receiver_concept = lexec::receiver_t;
    void set_value() && noexcept { flag->store(true); }
    std::atomic<bool> *flag;
};
struct Run {
    using Join = decltype(lexec::connect(std::declval<lexec::counting_scope &>().join(), Joined{}));
    Run(lrclexec::TimerScheduler const &scheduler, Statistics &statistics)
        : join{lexec::connect(scope.join(), Joined{&joined})} {
        auto resource = std::make_shared<Resource>(statistics);
        lifetime = resource;
        auto work =
            lrclexec::schedule_after(scheduler, 24h) | lexec::then([resource]() noexcept {}) |
            lexec::let_stopped([scheduler, resource, &statistics] {
                auto cleanup = lrclexec::schedule_after(scheduler, 10ms) |
                               lexec::then([resource, &statistics]() noexcept { ++statistics.cleanups; });
                return lexec::write_env(std::move(cleanup),
                                        lexec::prop{lexec::get_stop_token, lexec::never_stop_token{}});
            }) |
            lexec::upon_error([&statistics](std::exception_ptr) noexcept { ++statistics.errors; });
        lexec::spawn(std::move(work), scope.get_token());
        ++statistics.starts;
    }
    void stop() noexcept {
        if (stopping)
            return;
        stopping = true;
        scope.close();
        lexec::start(join);
        scope.request_stop(); // Queues cancellation; never waits for join.
    }
    bool drained() const noexcept { return joined.load(); }
    bool resourceReleased() const noexcept { return lifetime.expired(); }

  private:
    lexec::counting_scope scope;
    std::atomic<bool> joined{false};
    Join join;
    std::weak_ptr<Resource> lifetime;
    bool stopping = false;
};

// External ownership avoids node -> scheduler -> node ownership cycles.
// All callbacks in this finite example run in the scheduler's callback group.
struct Controller {
    explicit Controller(std::shared_ptr<Node> node_)
        : managedNode{std::move(node_)}, scheduling{managedNode} {}
    CallbackReturn activate() {
        if (run and not run->drained())
            return CallbackReturn::FAILURE;
        if (rejectNextActivation) {
            rejectNextActivation = false;
            return CallbackReturn::FAILURE;
        }
        run.reset();
        try {
            run = std::make_unique<Run>(scheduling, counts);
        } catch (...) {
            ++counts.errors;
            return CallbackReturn::FAILURE;
        }
        return CallbackReturn::SUCCESS;
    }
    CallbackReturn stop() noexcept {
        if (run)
            run->stop();
        return CallbackReturn::SUCCESS;
    }
    CallbackReturn release() {
        if (run and not run->drained())
            return CallbackReturn::FAILURE;
        run.reset();
        return CallbackReturn::SUCCESS;
    }
    bool drained() const noexcept { return not run or run->drained(); }
    bool resourceReleased() const noexcept { return not run or run->resourceReleased(); }
    std::shared_ptr<Node> const &node() const noexcept { return managedNode; }
    lrclexec::TimerScheduler const &scheduler() const noexcept { return scheduling; }
    Statistics const &statistics() const noexcept { return counts; }

  private:
    std::shared_ptr<Node> managedNode;
    lrclexec::TimerScheduler scheduling;
    Statistics counts;
    std::unique_ptr<Run> run;
    bool rejectNextActivation = true; // Demonstrate retry after a recoverable failure.
};
std::shared_ptr<Controller> makeController(std::shared_ptr<Node> node) {
    auto controller = std::make_shared<Controller>(std::move(node));
    auto const weak = std::weak_ptr<Controller>{controller};
    auto callback = [weak](auto method) {
        return [weak, method](rclcpp_lifecycle::State const &) {
            if (auto owner = weak.lock())
                return ((*owner).*method)();
            return CallbackReturn::FAILURE;
        };
    };
    controller->node()->register_on_activate(callback(&Controller::activate));
    controller->node()->register_on_deactivate(callback(&Controller::stop));
    controller->node()->register_on_cleanup(callback(&Controller::release));
    controller->node()->register_on_shutdown(callback(&Controller::stop));
    controller->node()->register_on_error(callback(&Controller::stop));
    return controller;
}
struct Spinning {
    explicit Spinning(rclcpp::Executor &executor_)
        : executor{executor_}, thread{[&executor_] {
              try {
                  executor_.spin();
              } catch (std::exception const &error) {
                  std::cerr << "executor failure: " << error.what() << '\n';
                  std::_Exit(125);
              }
          }} {}
    ~Spinning() {
        executor.cancel();
        thread.join();
    }
    Spinning(Spinning const &) = delete;
    Spinning &operator=(Spinning const &) = delete;
    Spinning(Spinning &&) = delete;
    Spinning &operator=(Spinning &&) = delete;
    rclcpp::Executor &executor;
    std::thread thread;
};
template <class Function> auto onExecutor(lrclexec::TimerScheduler const &scheduler, Function function) {
    using Result = decltype(function());
    auto promise = std::promise<Result>{};
    auto future = promise.get_future();
    scheduler.executionContext()->post([&] {
        try {
            if constexpr (std::is_void_v<Result>) {
                function();
                promise.set_value();
            } else
                promise.set_value(function());
        } catch (...) {
            promise.set_exception(std::current_exception());
        }
    });
    if (future.wait_for(3s) != std::future_status::ready) {
        std::cerr << "lifecycle transition/cleanup stalled\n";
        std::_Exit(124); // Keep pending scopes and callback borrows alive.
    }
    return future.get();
}
void awaitDrain(std::shared_ptr<Controller> const &controller) {
    auto const deadline = std::chrono::steady_clock::now() + 3s;
    while (not onExecutor(controller->scheduler(), [&] { return controller->drained(); })) {
        if (std::chrono::steady_clock::now() >= deadline) {
            std::cerr << "lifecycle scope failed to drain\n";
            std::_Exit(124);
        }
        std::this_thread::sleep_for(1ms);
    }
    onExecutor(controller->scheduler(), [&] {
        check(controller->resourceReleased(), "scope joined before operation resources were released");
        check(controller->statistics().live == 0, "old generation is still alive");
    });
}
std::unique_ptr<rclcpp::Executor> makeExecutor(std::string const &mode) {
    if (mode == "events")
        return std::make_unique<rclcpp::experimental::executors::EventsExecutor>();
    if (mode == "multi")
        return std::make_unique<rclcpp::executors::MultiThreadedExecutor>(rclcpp::ExecutorOptions{}, 4);
    if (mode == "single")
        return std::make_unique<rclcpp::executors::SingleThreadedExecutor>();
    throw std::invalid_argument{"executor must be single, multi or events"};
}
void deactivate(std::shared_ptr<Controller> const &controller) {
    onExecutor(controller->scheduler(), [&] {
        auto const &node = controller->node();
        check(node->deactivate().label() == "inactive", "deactivate failed");
        check(not controller->drained(), "deactivate blocked until join");
        auto result = CallbackReturn::SUCCESS;
        check(node->activate(result).label() == "inactive" and result == CallbackReturn::FAILURE,
              "reactivation overlapped the draining generation");
        check(node->cleanup(result).label() == "inactive" and result == CallbackReturn::FAILURE,
              "cleanup released a pending scope");
    });
    awaitDrain(controller);
}
void prepareShutdown(std::shared_ptr<Controller> const &controller, std::string const &state) {
    onExecutor(controller->scheduler(), [&] {
        auto const &node = controller->node();
        check(node->cleanup().label() == "unconfigured", "cleanup failed after drain");
        if (state != "unconfigured")
            check(node->configure().label() == "inactive", "reconfigure failed");
        if (state == "active")
            check(node->activate().label() == "active", "final activation failed");
    });
    onExecutor(controller->scheduler(),
               [&] { check(controller->node()->shutdown().label() == "finalized", "shutdown failed"); });
    awaitDrain(controller);
}
void drainAfterFailure(std::shared_ptr<Controller> const &controller) noexcept {
    try {
        onExecutor(controller->scheduler(), [&] { controller->stop(); });
        awaitDrain(controller);
    } catch (...) {
        std::cerr << "failed to drain lifecycle work after a control exception\n";
        std::_Exit(125);
    }
}
void exerciseControlFailure(std::shared_ptr<Controller> const &controller) {
    onExecutor(controller->scheduler(), [&] {
        check(controller->node()->activate().label() == "active", "failure exercise activation failed");
    });
    auto observed = false;
    try {
        onExecutor(controller->scheduler(), [] { throw std::runtime_error{"injected control failure"}; });
    } catch (std::runtime_error const &error) {
        if (std::string{error.what()} != "injected control failure")
            throw;
        observed = true;
        drainAfterFailure(controller);
    }
    check(observed, "control failure was not propagated");
    onExecutor(controller->scheduler(), [&] {
        check(controller->node()->deactivate().label() == "inactive", "failure recovery deactivate failed");
    });
}
void demonstrate(std::string const &mode, int const cycles, std::string const &shutdownFrom) {
    std::weak_ptr<Node> lifetime;
    {
        // No remote transition services: the application serializes its policy.
        auto node = std::make_shared<Node>("lrclexec_lifecycle_session", rclcpp::NodeOptions{}, false);
        lifetime = node;
        auto controller = makeController(node);
        auto executor = makeExecutor(mode);
        executor->add_node(node->get_node_base_interface());
        auto spinning = Spinning{*executor};
        try {
            onExecutor(controller->scheduler(), [&] {
                check(node->configure().label() == "inactive", "configure failed");
                auto result = CallbackReturn::SUCCESS;
                check(node->activate(result).label() == "inactive" and result == CallbackReturn::FAILURE,
                      "failed activation did not stay inactive");
            });
            exerciseControlFailure(controller);
            for (int cycle = 0; cycle < cycles; ++cycle) {
                onExecutor(controller->scheduler(),
                           [&] { check(node->activate().label() == "active", "activate failed"); });
                // Ordered behind Run::begin(): deactivate a genuinely armed task.
                deactivate(controller);
            }
            prepareShutdown(controller, shutdownFrom);
            onExecutor(controller->scheduler(), [&] {
                auto const &statistics = controller->statistics();
                auto const expected = cycles + 1 + (shutdownFrom == "active" ? 1 : 0);
                check(statistics.starts == expected and statistics.cleanups == expected and
                          statistics.errors == 0 and statistics.peak == 1,
                      "generation accounting failed");
            });
            std::cout << mode << ": " << cycles << " activation cycles; shutdown from " << shutdownFrom
                      << "; asynchronous cleanup and resource release verified\n";
        } catch (...) {
            auto const failure = std::current_exception();
            drainAfterFailure(controller);
            std::rethrow_exception(failure);
        }
        // Spinning is destroyed before controller/scheduler/node owners.
    }
    check(lifetime.expired(), "lifecycle example created an ownership cycle");
}
} // namespace

int main(int argc, char **argv) {
    try {
        auto mode = std::string{"single"};
        auto cycles = 2;
        for (int i = 1; i < argc; i += 2) {
            if (i + 1 == argc)
                throw std::invalid_argument{"arguments need a value"};
            auto const key = std::string{argv[i]};
            if (key == "--executor")
                mode = argv[i + 1];
            else if (key == "--cycles") {
                auto used = std::size_t{0};
                auto const value = std::string{argv[i + 1]};
                cycles = std::stoi(value, &used);
                if (used != value.size())
                    throw std::invalid_argument{"invalid cycle count"};
            } else
                throw std::invalid_argument{"unknown argument"};
        }
        if (cycles < 1 or cycles > 10000)
            throw std::invalid_argument{"cycles must be 1..10000"};
        if (mode != "single" and mode != "multi" and mode != "events")
            throw std::invalid_argument{"executor must be single, multi or events"};
        if (mode == "events")
            std::cerr << "experimental diagnostic: Jazzy 28.1.22 may stall when timer handles are reused\n";
        rclcpp::init(0, nullptr, rclcpp::InitOptions{}, rclcpp::SignalHandlerOptions::None);
        for (auto const &state : {"unconfigured", "inactive", "active"})
            demonstrate(mode, cycles, state);
        rclcpp::shutdown();
        return 0;
    } catch (std::exception const &error) {
        std::cerr << error.what() << '\n';
        return 1;
    }
}
