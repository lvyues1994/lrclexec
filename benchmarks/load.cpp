#include "Metrics.h"
#include <cstdlib>
#include <example_interfaces/srv/add_two_ints.hpp>
#include <future>
#include <iostream>
#include <lrclexec/Service.h>
#include <lrclexec/Topic.h>
#include <rclcpp/experimental/executors/events_executor/events_executor.hpp>
#include <rclcpp/rclcpp.hpp>
#include <std_msgs/msg/int64.hpp>
#include <thread>
#include <vector>

using namespace std::chrono_literals;
using measurement::Clock;
using measurement::Kind;
using Service = example_interfaces::srv::AddTwoInts;
using Message = std_msgs::msg::Int64;

namespace {
struct Options {
    std::string executor = "single", mode = "schedule";
    int seconds = 10, warmup = 2, producers = 4, window = 256;
};
Options arguments(int const argc, char **const argv) {
    auto result = Options{};
    for (int i = 1; i < argc; i += 2) {
        if (i + 1 == argc)
            throw std::invalid_argument{"arguments require a value"};
        auto const key = std::string{argv[i]};
        auto const value = std::string{argv[i + 1]};
        if (key == "--executor")
            result.executor = value;
        else if (key == "--mode")
            result.mode = value;
        else {
            auto used = std::size_t{0};
            auto const number = std::stoi(value, &used);
            if (used != value.size())
                throw std::invalid_argument{"invalid numeric argument"};
            if (key == "--seconds")
                result.seconds = number;
            else if (key == "--warmup")
                result.warmup = number;
            else if (key == "--producers")
                result.producers = number;
            else if (key == "--window")
                result.window = number;
            else
                throw std::invalid_argument{"unknown argument"};
        }
    }
    if ((result.executor != "single" and result.executor != "multi" and result.executor != "events") or
        (result.mode != "post" and result.mode != "schedule" and result.mode != "mixed") or
        result.seconds < 1 or result.seconds > 86400 or result.warmup < 0 or result.warmup > 60 or
        result.producers < 1 or result.producers > 64 or result.window < 1 or result.window > 4096)
        throw std::invalid_argument{"invalid executor, mode or numeric range"};
    return result;
}
// Join even if subsequent thread creation or workload construction throws.
struct Threads {
    ~Threads() { join(); }
    Threads() = default;
    Threads(Threads const &) = delete;
    Threads &operator=(Threads const &) = delete;
    Threads(Threads &&) = delete;
    Threads &operator=(Threads &&) = delete;
    void join() {
        for (auto &thread : threads)
            if (thread.joinable())
                thread.join();
    }
    std::vector<std::thread> threads;
};
struct Spinning {
    explicit Spinning(rclcpp::Executor &executor_)
        : executor{executor_}, thread{[&executor_] {
              try {
                  executor_.spin();
              } catch (std::exception const &error) {
                  std::cerr << "executor failed: " << error.what() << '\n';
                  std::_Exit(125);
              }
          }} {}
    ~Spinning() { stop(); }
    Spinning(Spinning const &) = delete;
    Spinning &operator=(Spinning const &) = delete;
    Spinning(Spinning &&) = delete;
    Spinning &operator=(Spinning &&) = delete;
    void stop() {
        if (thread.joinable()) {
            executor.cancel();
            thread.join();
        }
    }
    rclcpp::Executor &executor;
    std::thread thread;
};
struct Observing {
    template <class Function>
    explicit Observing(Function function)
        : thread{[this, function = std::move(function)] { function(finished); }} {}
    ~Observing() { stop(); }
    Observing(Observing const &) = delete;
    Observing &operator=(Observing const &) = delete;
    Observing(Observing &&) = delete;
    Observing &operator=(Observing &&) = delete;
    void stop() {
        finished.store(true);
        if (thread.joinable())
            thread.join();
    }

  private:
    std::atomic<bool> finished{false};
    std::thread thread;
};
struct PromiseReceiver {
    using receiver_concept = lexec::receiver_t;
    void set_value() && noexcept { deliver(1); }
    void set_stopped() && noexcept { deliver(0); }
    void set_error(std::exception_ptr) && noexcept { deliver(-1); }
    void deliver(int const value) noexcept {
        if (delivered->exchange(true)) {
            std::cerr << "duplicate timer completion\n";
            std::_Exit(125);
        }
        promise->set_value(value);
    }
    std::promise<int> *promise;
    std::atomic<bool> *delivered;
};
int cancelTimer(lrclexec::TimerScheduler const &scheduler, bool const race, measurement::Ticket &ticket) {
    auto source = lexec::inplace_stop_source{};
    auto promise = std::promise<int>{};
    auto delivered = std::atomic<bool>{false};
    auto future = promise.get_future();
    auto operation = lexec::connect(lexec::write_env(lrclexec::schedule_after(scheduler, race ? 1ms : 10s),
                                                     lexec::prop{lexec::get_stop_token, source.get_token()}),
                                    PromiseReceiver{&promise, &delivered});
    lexec::start(operation);
    try {
        (void)lexec::sync_wait(lexec::schedule(scheduler)); // The timer has been armed.
    } catch (...) {
        source.request_stop();
        if (future.wait_for(3s) != std::future_status::ready) {
            std::cerr << "timer did not drain after the arm barrier failed\n";
            std::_Exit(124); // Preserve the token and receiver's borrowed objects.
        }
        throw;
    }
    if (not race)
        ticket.start = Clock::now(); // Cancellation-to-observed-completion.
    source.request_stop();
    if (future.wait_for(3s) != std::future_status::ready) {
        std::cerr << "timer cancellation did not drain\n";
        std::_Exit(124); // Keep the pending operation and borrowed receiver alive.
    }
    auto const result = future.get();
    return not race and result != 0 ? -1 : result;
}
int mixed(lrclexec::TimerScheduler const &scheduler, std::shared_ptr<rclcpp::Client<Service>> const &client,
          std::string const &topic, measurement::Ticket &ticket) {
    if (ticket.kind == Kind::timerCancel or ticket.kind == Kind::timerRace)
        return cancelTimer(scheduler, ticket.kind == Kind::timerRace, ticket);
    if (ticket.kind == Kind::schedule)
        return lexec::sync_wait(lexec::schedule(scheduler)) ? 1 : -1;
    auto timeout = lrclexec::schedule_after(scheduler, 2s) | lexec::then([]() noexcept { return -2; });
    if (ticket.kind == Kind::service) {
        auto request = Service::Request{};
        request.a = 19;
        request.b = 23;
        auto work = lrclexec::call_service(scheduler, client, request) |
                    lexec::then([](auto response) noexcept { return response->sum == 42 ? 1 : -1; });
        auto result = lexec::sync_wait(lexec::when_any(std::move(work), std::move(timeout)));
        return result ? std::get<0>(*result) : -1;
    }
    auto work = lrclexec::wait_message<Message>(scheduler, topic) |
                lexec::then([](auto message) noexcept { return message->data == 42 ? 1 : -1; });
    auto result = lexec::sync_wait(lexec::when_any(std::move(work), std::move(timeout)));
    return result ? std::get<0>(*result) : -1;
}
void sample(measurement::Accounting const &accounting, Clock::time_point const start,
            char const *const phase) {
    auto const totals = accounting.snapshot();
    auto const rss = measurement::residentKiB();
    std::cout << "{\"type\":\"sample\",\"phase\":\"" << phase
              << "\",\"seconds\":" << std::chrono::duration<double>(Clock::now() - start).count()
              << ",\"admitted\":" << totals.admitted
              << ",\"completed\":" << totals.values + totals.stopped + totals.errors
              << ",\"inflight\":" << totals.inflight << ",\"rss_kib\":";
    if (rss)
        std::cout << *rss;
    else
        std::cout << "null";
    std::cout << "}\n" << std::flush;
}
bool phase(Options const &options, int const seconds, char const *const name,
           lrclexec::TimerScheduler const &scheduler,
           std::vector<std::shared_ptr<rclcpp::Client<Service>>> const &clients) {
    auto accounting = measurement::Accounting{static_cast<std::uint64_t>(options.window)};
    auto scope = lexec::counting_scope{};
    auto const start = Clock::now();
    auto const deadline = start + std::chrono::seconds{seconds};
    auto observer = Observing{[&](std::atomic<bool> const &finished) {
        auto nextSample = start;
        auto lastProgress = start;
        auto lastCount = std::uint64_t{0};
        while (not finished.load()) {
            auto const now = Clock::now();
            auto const totals = accounting.snapshot();
            auto const completed = totals.values + totals.stopped + totals.errors;
            if (completed != lastCount or totals.inflight == 0)
                lastProgress = now;
            lastCount = completed;
            if (now >= nextSample) {
                sample(accounting, start, name);
                nextSample = now + 1s;
            }
            if (now > deadline + 10s or now > lastProgress + 10s) {
                std::cerr << "load watchdog: pending work did not complete\n";
                std::_Exit(124);
            }
            std::this_thread::sleep_for(50ms);
        }
    }};
    auto producers = Threads{};
    try {
        for (int producer = 0; producer < options.producers; ++producer)
            producers.threads.emplace_back([&, producer] {
                try {
                    auto iteration = std::uint64_t{0};
                    constexpr std::array<Kind, 5> kinds{Kind::schedule, Kind::timerCancel, Kind::timerRace,
                                                        Kind::service, Kind::topic};
                    while (true) {
                        auto const kind = options.mode == "post"       ? Kind::post
                                          : options.mode == "schedule" ? Kind::schedule
                                                                       : kinds[iteration++ % kinds.size()];
                        auto ticket = accounting.admit(kind, deadline);
                        if (not ticket)
                            break;
                        ticket->start = Clock::now();
                        try {
                            if (options.mode == "post")
                                scheduler.executionContext()->post(
                                    [&, ticket = *ticket] { accounting.finish(ticket, 1); });
                            else if (options.mode == "schedule") {
                                auto work =
                                    lexec::schedule(scheduler) |
                                    lexec::then(
                                        [&, ticket = *ticket]() noexcept { accounting.finish(ticket, 1); }) |
                                    lexec::upon_error([&, ticket = *ticket](std::exception_ptr) noexcept {
                                        accounting.finish(ticket, -1);
                                    }) |
                                    lexec::upon_stopped(
                                        [&, ticket = *ticket]() noexcept { accounting.finish(ticket, -1); });
                                lexec::spawn(std::move(work), scope.get_token());
                            } else {
                                auto const result =
                                    mixed(scheduler, clients.at(static_cast<std::size_t>(producer)),
                                          "load_topic_" + std::to_string(producer), *ticket);
                                accounting.finish(*ticket, result);
                                if (result < 0)
                                    std::cerr
                                        << "workload failure: phase=" << name << " producer=" << producer
                                        << " kind=" << measurement::names[static_cast<std::size_t>(kind)]
                                        << " outcome=" << result << " (-2=timeout, -1=invalid result)\n";
                            }
                        } catch (std::exception const &error) {
                            accounting.finish(*ticket, -1);
                            std::cerr << "workload exception: phase=" << name << " producer=" << producer
                                      << " kind=" << measurement::names[static_cast<std::size_t>(kind)]
                                      << " error=" << error.what() << '\n';
                        } catch (...) {
                            accounting.finish(*ticket, -1);
                            std::cerr << "unknown workload exception\n";
                        }
                    }
                } catch (...) {
                    accounting.fail();
                }
            });
    } catch (...) {
        accounting.fail();
    }
    producers.join();
    scope.close();
    (void)lexec::sync_wait(scope.join());
    (void)lexec::sync_wait(lexec::schedule(scheduler)); // Ordered after all submitted post callbacks.
    auto const end = Clock::now();
    observer.stop();
    sample(accounting, start, name);
    auto const totals = accounting.snapshot();
    auto const completed = totals.values + totals.stopped + totals.errors;
    auto const passed = not accounting.failedRun() and totals.admitted == completed and completed > 0 and
                        totals.inflight == 0 and totals.peak <= static_cast<std::uint64_t>(options.window);
    auto const elapsed = std::chrono::duration<double>(end - start).count();
    std::cout << "{\"type\":\"summary\",\"phase\":\"" << name << "\",\"executor\":\"" << options.executor
              << "\",\"mode\":\"" << options.mode << "\",\"producers\":" << options.producers
              << ",\"window\":" << options.window << ",\"requested_seconds\":" << seconds
              << ",\"elapsed_seconds\":" << elapsed << ",\"after_deadline_seconds\":"
              << std::max(0.0, std::chrono::duration<double>(end - deadline).count())
              << ",\"admitted\":" << totals.admitted << ",\"values\":" << totals.values
              << ",\"stopped\":" << totals.stopped << ",\"errors\":" << totals.errors
              << ",\"duplicates\":" << totals.duplicates << ",\"inflight\":" << totals.inflight
              << ",\"peak_inflight\":" << totals.peak
              << ",\"throughput_per_second\":" << static_cast<double>(completed) / elapsed
              << ",\"passed\":" << (passed ? "true" : "false") << ",\"latency\":{";
    auto first = true;
    for (std::size_t i = 0; i < totals.latency.size(); ++i) {
        auto const &histogram = totals.latency[i];
        if (histogram.count() == 0)
            continue;
        if (not first)
            std::cout << ',';
        first = false;
        std::cout << '"' << measurement::names[i] << "\":{\"count\":" << histogram.count()
                  << ",\"p50_upper_ns\":" << histogram.percentile(0.50)
                  << ",\"p95_upper_ns\":" << histogram.percentile(0.95)
                  << ",\"p99_upper_ns\":" << histogram.percentile(0.99)
                  << ",\"max_ns\":" << histogram.maximum() << '}';
    }
    std::cout << "}}\n" << std::flush;
    return passed;
}
} // namespace

int main(int argc, char **argv) {
    try {
        auto const options = arguments(argc, argv);
        rclcpp::init(0, nullptr, rclcpp::InitOptions{}, rclcpp::SignalHandlerOptions::None);
        auto node = std::make_shared<rclcpp::Node>("lrclexec_load");
        auto peer = std::make_shared<rclcpp::Node>("lrclexec_load_peer");
        auto scheduler = lrclexec::TimerScheduler{node};
        std::unique_ptr<rclcpp::Executor> executor;
        if (options.executor == "events")
            executor = std::make_unique<rclcpp::experimental::executors::EventsExecutor>();
        else if (options.executor == "multi")
            executor =
                std::make_unique<rclcpp::executors::MultiThreadedExecutor>(rclcpp::ExecutorOptions{}, 4);
        else
            executor = std::make_unique<rclcpp::executors::SingleThreadedExecutor>();
        executor->add_node(node);
        executor->add_node(peer);
        std::vector<std::shared_ptr<rclcpp::Client<Service>>> clients;
        std::vector<rclcpp::Publisher<Message>::SharedPtr> publishers;
        rclcpp::Service<Service>::SharedPtr service;
        rclcpp::TimerBase::SharedPtr publishTimer;
        if (options.mode == "mixed") {
            service =
                peer->create_service<Service>("load_service", [](Service::Request::SharedPtr request,
                                                                 Service::Response::SharedPtr response) {
                    response->sum = request->a + request->b;
                });
            for (int i = 0; i < options.producers; ++i) {
                clients.push_back(node->create_client<Service>("load_service"));
                publishers.push_back(peer->create_publisher<Message>("load_topic_" + std::to_string(i), 1));
            }
            publishTimer = peer->create_wall_timer(2ms, [&] {
                auto message = Message{};
                message.data = 42;
                for (auto const &publisher : publishers)
                    publisher->publish(message);
            });
        }
        auto spinner = Spinning{*executor};
        auto passed = options.warmup == 0 or phase(options, options.warmup, "warmup", scheduler, clients);
        if (passed)
            passed = phase(options, options.seconds, "measure", scheduler, clients);
        if (publishTimer) {
            publishTimer->cancel();
            publishTimer.reset();
        }
        spinner.stop();
        // Client/publisher/node owners outlive the executor's collection thread.
        rclcpp::shutdown();
        return passed ? 0 : 1;
    } catch (std::exception const &error) {
        std::cerr << error.what() << '\n';
        return 1;
    }
}
