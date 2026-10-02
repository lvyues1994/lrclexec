#include "Metrics.h"
#include <example_interfaces/action/fibonacci.hpp>
#include <iostream>
#include <lrclexec/ActionServer.h>
#include <lrclexec/ExecuteAction.h>
#include <rclcpp/rclcpp.hpp>
#include <rclcpp_action/create_client.hpp>
#include <thread>

using namespace std::chrono_literals;
using Action = example_interfaces::action::Fibonacci;
using Handle = rclcpp_action::ServerGoalHandle<Action>;
using measurement::Clock;
namespace {
[[noreturn]] void fail(char const *stage) {
    std::cerr << "action load failed: " << stage << '\n' << std::flush;
    std::_Exit(125); // Preserve in-flight borrowed tokens; runner records the failed process.
}
void check(bool condition, char const *stage) {
    if (not condition)
        fail(stage);
}
template <class Predicate> void wait(Predicate predicate, char const *stage) {
    auto const deadline = Clock::now() + 5s;
    while (not predicate()) {
        if (Clock::now() >= deadline)
            fail(stage);
        std::this_thread::sleep_for(1ms);
    }
}
struct Statistics {
    std::atomic<unsigned long> admitted{0}, completed{0}, duplicates{0}, feedback{0}, cycles{0};
    std::atomic<unsigned> inflight{0}, peak{0};
    std::array<std::atomic<unsigned long>, 4> outcomes{}; // value, stopped, rejected, aborted
    std::mutex mutex;
    measurement::Histogram latency;
    void start() {
        ++admitted;
        auto const live = ++inflight;
        auto old = peak.load();
        while (old < live and not peak.compare_exchange_weak(old, live)) {
        }
    }
};
struct Result {
    std::atomic<bool> done{false}, delivered{false};
    int outcome = -1, payload = -1;
    std::atomic<unsigned> feedback{0}, cancelResponses{0};
    std::atomic<int> cancelCode{-1};
    Clock::time_point started = Clock::now();
};
void finish(Statistics &stats, Result &state, int outcome, int payload = -1) noexcept {
    if (state.delivered.exchange(true)) {
        ++stats.duplicates;
        fail("duplicate completion");
    }
    state.outcome = outcome;
    state.payload = payload;
    {
        auto lock = std::lock_guard{stats.mutex};
        stats.latency.add(static_cast<std::uint64_t>(
            std::chrono::duration_cast<std::chrono::nanoseconds>(Clock::now() - state.started).count()));
    }
    check(outcome >= 0 and outcome < 4, "unexpected completion");
    ++stats.outcomes[static_cast<std::size_t>(outcome)];
    --stats.inflight;
    ++stats.completed;
    state.done = true;
}
struct Errors {
    void operator()(lrclexec::ActionError<Action> error) const noexcept {
        int outcome = error.kind == lrclexec::ActionErrorKind::rejected  ? 2
                      : error.kind == lrclexec::ActionErrorKind::aborted ? 3
                                                                         : -1;
        finish(*stats, *state, outcome,
               error.result and not error.result->sequence.empty() ? error.result->sequence[0] : -1);
    }
    void operator()(std::exception_ptr) const noexcept { fail("unexpected exception completion"); }
    Statistics *stats;
    std::shared_ptr<Result> state;
};
class Attempt {
  public:
    Attempt(lrclexec::TimerScheduler const &scheduler, rclcpp_action::Client<Action>::SharedPtr client,
            lexec::counting_scope &scope, Statistics &stats, int id)
        : state{std::make_shared<Result>()} {
        auto options = lrclexec::ActionOptions<Action>{};
        options.feedback = [state = state, &stats, id](auto feedback) {
            check(not state->done.load(), "feedback after terminal");
            check(feedback->sequence == std::vector<std::int32_t>{id}, "feedback payload mismatch");
            ++state->feedback;
            ++stats.feedback;
        };
        options.cancelResponse = [state = state](auto response) {
            state->cancelCode = response->return_code;
            ++state->cancelResponses;
        };
        auto goal = Action::Goal{};
        goal.order = id;
        stats.start();
        auto work =
            lexec::write_env(lrclexec::execute_action(scheduler, std::move(client), goal, std::move(options)),
                             lexec::prop{lexec::get_stop_token, stop.get_token()}) |
            lexec::then([state = state, &stats](auto result) noexcept {
                check(result and result->sequence.size() == 1, "missing result payload");
                finish(stats, *state, 0, result->sequence[0]);
            }) |
            lexec::upon_error(Errors{&stats, state}) |
            lexec::upon_stopped([state = state, &stats]() noexcept { finish(stats, *state, 1); });
        lexec::spawn(std::move(work), scope.get_token());
    }
    ~Attempt() {
        stop.request_stop();
        wait([&] { return state->done.load(); }, "attempt destruction drain");
    }
    Attempt(Attempt const &) = delete;
    Attempt &operator=(Attempt const &) = delete;
    void expect(int outcome, int payload = -1) {
        wait([&] { return state->done.load(); }, "terminal response");
        check(state->outcome == outcome, "wrong terminal channel");
        if (payload >= 0)
            check(state->payload == payload, "wrong terminal payload");
    }
    std::shared_ptr<Result> state;
    lexec::inplace_stop_source stop;
};
struct Business {
    std::atomic<int> live{0}, peak{0}, overlaps{0}, factories{0}, activeId{-1}, cleaning{0}, nativeId{-1};
    std::atomic<bool> cleanupAllowed{true}, nativeAllowed{false};
    std::weak_ptr<Handle> feedbackHandle;
    std::shared_ptr<Handle> nativeHandle;
};
struct Lease {
    explicit Lease(Business &business_) : business{business_} {
        auto const count = ++business.live;
        if (count != 1)
            ++business.overlaps;
        auto old = business.peak.load();
        while (old < count and not business.peak.compare_exchange_weak(old, count)) {
        }
    }
    ~Lease() { --business.live; }
    Business &business;
};
struct Factory {
    auto operator()(std::shared_ptr<Handle> handle) const {
        ++business->factories;
        auto const id = handle->get_goal()->order;
        business->activeId = id;
        business->feedbackHandle = handle;
        auto lease = std::make_shared<Lease>(*business);
        auto result = std::make_shared<Action::Result>();
        result->sequence = {id};
        auto const mode = id % 10;
        return lrclexec::schedule_after(scheduler, mode == 2 or mode == 3 or mode == 6 ? 1h : 2ms) |
               lexec::then([lease, result]() noexcept { return result; }) |
               lexec::let_stopped([scheduler = scheduler, business = business, lease] {
                   ++business->cleaning;
                   auto work = lexec::repeat_until(lrclexec::schedule_after(scheduler, 2ms) |
                                                   lexec::then([business]() noexcept {
                                                       return business->cleanupAllowed.load();
                                                   })) |
                               lexec::let_value([lease]() noexcept { return lexec::just_stopped(); });
                   return lexec::write_env(std::move(work),
                                           lexec::prop{lexec::get_stop_token, lexec::never_stop_token{}});
               });
    }
    lrclexec::TimerScheduler scheduler;
    Business *business;
};
using Server = decltype(lrclexec::make_action_server_preempt<Action>(std::declval<lrclexec::TimerScheduler>(),
                                                                     std::declval<lexec::counting_scope &>(),
                                                                     "", std::declval<Factory>()));
struct Lane {
    explicit Lane(int index)
        : node{std::make_shared<rclcpp::Node>("action_load_" + std::to_string(index))}, scheduler{node},
          name{"load_action_" + std::to_string(index)},
          server{lrclexec::make_action_server_preempt<Action>(scheduler, serverScope, name,
                                                              Factory{scheduler, &business})},
          client{rclcpp_action::create_client<Action>(node, name)},
          protocol{rclcpp_action::create_client<Action>(node, name + "_protocol")} {
        timer = node->create_wall_timer(2ms, [this] { tick(); }, scheduler.callbackGroup());
    }
    void createProtocol() {
        native = rclcpp_action::create_server<Action>(
            node, name + "_protocol",
            [](auto const &, auto const &goal) {
                return goal->order % 10 == 7 ? rclcpp_action::GoalResponse::REJECT
                                             : rclcpp_action::GoalResponse::ACCEPT_AND_EXECUTE;
            },
            [](auto) { return rclcpp_action::CancelResponse::REJECT; },
            [this](std::shared_ptr<Handle> handle) {
                if (handle->get_goal()->order % 10 == 8) {
                    auto result = std::make_shared<Action::Result>();
                    result->sequence = {handle->get_goal()->order};
                    handle->abort(result);
                } else {
                    business.nativeId = handle->get_goal()->order;
                    business.nativeHandle = std::move(handle);
                }
            },
            rcl_action_server_get_default_options(), scheduler.callbackGroup());
    }
    void tick() {
        if (auto handle = business.feedbackHandle.lock();
            handle and handle->is_active() and handle->get_goal()->order % 10 == 6) {
            auto message = std::make_shared<Action::Feedback>();
            message->sequence = {handle->get_goal()->order};
            handle->publish_feedback(message);
        }
        if (business.nativeHandle and business.nativeAllowed.load()) {
            auto handle = std::exchange(business.nativeHandle, {});
            auto result = std::make_shared<Action::Result>();
            result->sequence = {handle->get_goal()->order};
            handle->succeed(result);
        }
    }
    void cycle(Statistics &stats, int serial) {
        auto const base = serial * 10;
        {
            auto success = Attempt{scheduler, client, clientScope, stats, base + 1};
            success.expect(0, base + 1);
        }
        wait([&] { return business.live == 0; }, "success resource release");
        {
            auto cancel = Attempt{scheduler, client, clientScope, stats, base + 2};
            wait([&] { return business.activeId == base + 2; }, "cancel factory start");
            cancel.stop.request_stop();
            cancel.expect(1);
        }
        wait([&] { return business.live == 0; }, "canceled resource release");
        auto const factories = business.factories.load();
        auto const cleaning = business.cleaning.load();
        business.cleanupAllowed = false;
        {
            auto first = Attempt{scheduler, client, clientScope, stats, base + 3};
            wait([&] { return business.activeId == base + 3; }, "preemption factory start");
            auto second = Attempt{scheduler, client, clientScope, stats, base + 4};
            wait([&] { return business.cleaning > cleaning; }, "preemption cleanup start");
            auto third = Attempt{scheduler, client, clientScope, stats, base + 5};
            second.expect(3); // Third was accepted while first still owns its resource.
            business.cleanupAllowed = true;
            first.expect(3);
            third.expect(0, base + 5);
        }
        wait([&] { return business.live == 0; }, "preemption resource release");
        check(business.factories == factories + 2, "superseded pending factory ran");
        {
            auto feedback = Attempt{scheduler, client, clientScope, stats, base + 6};
            wait([&] { return feedback.state->feedback > 0; }, "feedback observer");
            feedback.stop.request_stop();
            feedback.expect(1);
            auto const observed = feedback.state->feedback.load();
            (void)lexec::sync_wait(lrclexec::schedule_after(scheduler, 3ms));
            check(feedback.state->feedback == observed, "late feedback observer");
        }
        {
            auto rejected = Attempt{scheduler, protocol, clientScope, stats, base + 7};
            rejected.expect(2);
        }
        {
            auto aborted = Attempt{scheduler, protocol, clientScope, stats, base + 8};
            aborted.expect(3, base + 8);
        }
        business.nativeAllowed = false;
        {
            auto refused = Attempt{scheduler, protocol, clientScope, stats, base + 9};
            wait([&] { return business.nativeId == base + 9; }, "cancel refusal acceptance");
            refused.stop.request_stop();
            wait([&] { return refused.state->cancelResponses > 0 or refused.state->done; },
                 "cancel rejection response");
            check(not refused.state->done, "cancel refusal completed before remote terminal");
            check(refused.state->cancelCode == rclcpp_action::Client<Action>::CancelResponse::ERROR_REJECTED,
                  "cancel response was not REJECTED");
            business.nativeAllowed = true;
            refused.expect(0, base + 9);
        }
        wait([&] { return business.live == 0; }, "cycle resource release");
        ++stats.cycles;
    }
    void close() {
        business.cleanupAllowed = true;
        business.nativeAllowed = true;
        server.close();
        clientScope.close();
        clientScope.request_stop();
        serverScope.close();
        serverScope.request_stop();
        (void)lexec::sync_wait(clientScope.join());
        (void)lexec::sync_wait(serverScope.join());
        timer->cancel();
        check(business.live == 0 and business.overlaps == 0 and business.peak == 1,
              "business lifetime invariant");
    }
    rclcpp::Node::SharedPtr node;
    lrclexec::TimerScheduler scheduler;
    std::string name;
    Business business;
    lexec::counting_scope serverScope, clientScope;
    Server server;
    rclcpp_action::Client<Action>::SharedPtr client, protocol;
    rclcpp_action::Server<Action>::SharedPtr native;
    rclcpp::TimerBase::SharedPtr timer;
};
} // namespace

int main(int argc, char **argv) {
    try {
        std::string mode = "single";
        int seconds = 5, laneCount = 1;
        for (int i = 1; i < argc; i += 2) {
            if (i + 1 == argc)
                throw std::invalid_argument{"missing argument value"};
            std::string key = argv[i];
            if (key == "--executor")
                mode = argv[i + 1];
            else if (key == "--seconds")
                seconds = std::stoi(argv[i + 1]);
            else if (key == "--lanes")
                laneCount = std::stoi(argv[i + 1]);
            else
                throw std::invalid_argument{"unknown argument"};
        }
        if ((mode != "single" and mode != "multi") or seconds < 1 or seconds > 86400 or laneCount < 1 or
            laneCount > 4)
            throw std::invalid_argument{"invalid executor, duration or lanes"};
        rclcpp::init(0, nullptr, rclcpp::InitOptions{}, rclcpp::SignalHandlerOptions::None);
        std::unique_ptr<rclcpp::Executor> executor;
        if (mode == "multi")
            executor =
                std::make_unique<rclcpp::executors::MultiThreadedExecutor>(rclcpp::ExecutorOptions{}, 4);
        else
            executor = std::make_unique<rclcpp::executors::SingleThreadedExecutor>();
        std::vector<std::unique_ptr<Lane>> lanes;
        for (int i = 0; i < laneCount; ++i) {
            lanes.push_back(std::make_unique<Lane>(i));
            executor->add_node(lanes.back()->node);
        }
        auto stats = Statistics{};
        std::vector<std::thread> workers;
        auto spinner = std::thread{};
        try {
            spinner = std::thread{[&] {
                try {
                    executor->spin();
                } catch (...) {
                    fail("executor exception");
                }
            }};
            // After threads start, invariant failures exit without unwinding borrowed state.
            for (auto const &lane : lanes) {
                check(lane->client->wait_for_action_server(5s), "adapted discovery");
                check(not lane->protocol->action_server_is_ready(),
                      "protocol should initially be unavailable");
                lane->createProtocol();
                check(lane->protocol->wait_for_action_server(5s), "protocol discovery");
            }
            auto const start = Clock::now();
            auto const deadline = start + std::chrono::seconds{seconds};
            for (auto const &lane : lanes)
                workers.emplace_back([&, current = lane.get()] {
                    try {
                        int serial = 1;
                        do {
                            current->cycle(stats, serial++);
                        } while (Clock::now() < deadline);
                    } catch (...) {
                        fail("worker exception");
                    }
                });
            while (Clock::now() < deadline) {
                auto rss = measurement::residentKiB();
                std::cout << "{\"type\":\"sample\",\"seconds\":"
                          << std::chrono::duration<double>(Clock::now() - start).count()
                          << ",\"completed\":" << stats.completed << ",\"inflight\":" << stats.inflight
                          << ",\"rss_kib\":" << rss.value_or(0) << "}\n"
                          << std::flush;
                std::this_thread::sleep_for(1s);
            }
            for (auto &worker : workers)
                worker.join();
            auto const drainStart = Clock::now();
            for (auto const &lane : lanes)
                lane->close();
            auto const drainSeconds = std::chrono::duration<double>(Clock::now() - drainStart).count();
            executor->cancel();
            spinner.join();
            auto const elapsed = std::chrono::duration<double>(Clock::now() - start).count();
            check(stats.completed == stats.admitted and stats.inflight == 0 and stats.duplicates == 0,
                  "completion accounting");
            check(stats.admitted == stats.cycles * 9 and stats.peak <= static_cast<unsigned>(3 * laneCount),
                  "admission bound");
            std::cout << "{\"type\":\"summary\",\"executor\":\"" << mode << "\",\"seconds\":" << elapsed
                      << ",\"phase\":\"measure\",\"mode\":\"action\",\"producers\":" << laneCount
                      << ",\"window\":" << 3 * laneCount << ",\"throughput_per_second\":"
                      << static_cast<double>(stats.completed.load()) / elapsed
                      << ",\"recovered_servers\":" << laneCount << ",\"lanes\":" << laneCount
                      << ",\"cycles\":" << stats.cycles << ",\"admitted\":" << stats.admitted
                      << ",\"completed\":" << stats.completed << ",\"values\":" << stats.outcomes[0]
                      << ",\"stopped\":" << stats.outcomes[1] << ",\"rejected\":" << stats.outcomes[2]
                      << ",\"aborted\":" << stats.outcomes[3] << ",\"feedback\":" << stats.feedback
                      << ",\"duplicates\":" << stats.duplicates << ",\"inflight\":" << stats.inflight
                      << ",\"peak_inflight\":" << stats.peak
                      << ",\"business_peak_per_lane\":1,\"business_live_after_join\":0,\"join_seconds\":"
                      << drainSeconds << ",\"p99_upper_ns\":" << stats.latency.percentile(.99)
                      << ",\"max_ns\":" << stats.latency.maximum() << ",\"passed\":true}\n"
                      << std::flush;
        } catch (...) {
            fail("exception while thread owners are live");
        }
        rclcpp::shutdown();
        return 0;
    } catch (std::exception const &error) {
        std::cerr << error.what() << '\n';
        return 1;
    }
}
