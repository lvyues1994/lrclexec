#include <atomic>
#include <chrono>
#include <cstdlib>
#include <example_interfaces/action/fibonacci.hpp>
#include <future>
#include <iostream>
#include <lrclexec/ActionServer.h>
#include <lrclexec/ExecuteAction.h>
#include <lrclexec/SpinWithScope.h>
#include <mutex>
#include <rclcpp/rclcpp.hpp>
#include <rclcpp_action/rclcpp_action.hpp>
#include <stdexcept>
#include <thread>
#include <vector>

using namespace std::chrono_literals;
using Action = example_interfaces::action::Fibonacci;

void check(bool const condition, char const *const message) {
    if (not condition)
        throw std::runtime_error{message};
}

struct Fixture {
    Fixture() : scheduler{node} {
        executor.add_node(node);
        executor.add_node(serverNode);
        server = rclcpp_action::create_server<Action>(
            serverNode, "lrclexec_test_action",
            [this](auto const &, auto goal) {
                requests.fetch_add(1);
                if (delayAcceptance.load()) {
                    while (not allowAcceptance.load())
                        std::this_thread::sleep_for(1ms);
                }
                return goal->order == -1 ? rclcpp_action::GoalResponse::REJECT
                                         : rclcpp_action::GoalResponse::ACCEPT_AND_EXECUTE;
            },
            [this](auto) {
                cancellations.fetch_add(1);
                return acceptCancellation.load() ? rclcpp_action::CancelResponse::ACCEPT
                                                 : rclcpp_action::CancelResponse::REJECT;
            },
            [this](auto handle) {
                accepted.fetch_add(1);
                auto const started = std::chrono::steady_clock::now();
                auto timerSlot = std::make_shared<rclcpp::TimerBase::SharedPtr>();
                auto timer = serverNode->create_wall_timer(2ms, [this, handle, started, timerSlot] {
                    if (publishFeedback.exchange(false) and handle->is_executing()) {
                        auto feedback = std::make_shared<Action::Feedback>();
                        feedback->sequence = {7};
                        handle->publish_feedback(feedback);
                    }
                    if (not allowTerminal.load())
                        return;
                    auto const elapsed = std::chrono::steady_clock::now() - started;
                    auto result = std::make_shared<Action::Result>();
                    result->sequence = {42};
                    if (handle->is_canceling()) {
                        if (elapsed < cancelDelay)
                            return;
                        (*timerSlot)->cancel();
                        handle->canceled(result);
                        terminals.fetch_add(1);
                    } else if (handle->get_goal()->order == -2) {
                        (*timerSlot)->cancel();
                        handle->abort(result);
                        terminals.fetch_add(1);
                    } else if (elapsed >= std::chrono::milliseconds{handle->get_goal()->order}) {
                        (*timerSlot)->cancel();
                        handle->succeed(result);
                        terminals.fetch_add(1);
                    }
                });
                *timerSlot = timer;
                auto const lock = std::lock_guard<std::mutex>{timerMutex};
                timers.push_back({std::move(timer), timerSlot});
            });
        client = rclcpp_action::create_client<Action>(node, "lrclexec_test_action");
        startExecutor();
        check(client->wait_for_action_server(3s), "action server discovery failed");
    }
    ~Fixture() {
        allowAcceptance.store(true);
        stopExecutor();
        for (auto &timer : timers)
            timer.slot->reset();
    }
    auto action(int const milliseconds, lrclexec::ActionOptions<Action> options = {}) {
        auto goal = Action::Goal{};
        goal.order = milliseconds;
        return lrclexec::execute_action(scheduler, client, std::move(goal), std::move(options));
    }
    void startExecutor() {
        spinner = std::thread{[this] { executor.spin(); }};
    }
    void stopExecutor() {
        executor.cancel();
        if (spinner.joinable())
            spinner.join();
    }

    struct TimerEntry {
        rclcpp::TimerBase::SharedPtr timer;
        std::shared_ptr<rclcpp::TimerBase::SharedPtr> slot;
    };
    std::shared_ptr<rclcpp::Node> node = std::make_shared<rclcpp::Node>("lrclexec_test_client");
    std::shared_ptr<rclcpp::Node> serverNode = std::make_shared<rclcpp::Node>("lrclexec_test_server");
    lrclexec::TimerScheduler scheduler;
    rclcpp::executors::MultiThreadedExecutor executor{rclcpp::ExecutorOptions{}, 4};
    rclcpp_action::Server<Action>::SharedPtr server;
    rclcpp_action::Client<Action>::SharedPtr client;
    std::thread spinner;
    std::mutex timerMutex;
    std::vector<TimerEntry> timers;
    std::atomic<int> requests{0}, accepted{0}, cancellations{0}, terminals{0};
    std::atomic<bool> delayAcceptance{false}, allowAcceptance{false};
    std::atomic<bool> acceptCancellation{true}, allowTerminal{true}, publishFeedback{false};
    std::chrono::milliseconds cancelDelay{60};
};

struct StopExecutor {
    ~StopExecutor() { fixture.stopExecutor(); }
    Fixture &fixture;
};

template <class Predicate> void waitUntil(Predicate predicate) {
    auto const deadline = std::chrono::steady_clock::now() + 3s;
    while (not predicate()) {
        check(std::chrono::steady_clock::now() < deadline, "event wait timed out");
        std::this_thread::sleep_for(1ms);
    }
}

struct ReleaseGate {
    std::atomic<bool> &gate;
    ~ReleaseGate() { gate.store(true); }
};

void timerTests(Fixture &fixture) {
    static_assert(lexec::is_scheduler_v<lrclexec::TimerScheduler>);
    auto const mainThread = std::this_thread::get_id();
    auto const worker = lexec::sync_wait(lexec::schedule(fixture.scheduler) |
                                         lexec::then([] { return std::this_thread::get_id(); }));
    check(std::get<0>(*worker) != mainThread, "schedule did not enter executor");
    auto const started = std::chrono::steady_clock::now();
    (void)lexec::sync_wait(lrclexec::schedule_after(fixture.scheduler, 20ms));
    check(std::chrono::steady_clock::now() - started >= 20ms, "timer completed early");
    (void)lexec::sync_wait(lrclexec::schedule_after(fixture.scheduler, -1ms));

    auto source = lexec::inplace_stop_source{};
    source.request_stop();
    auto stopped = lexec::sync_wait(lexec::write_env(lrclexec::schedule_after(fixture.scheduler, 5s),
                                                     lexec::prop{lexec::get_stop_token, source.get_token()}));
    check(not stopped, "pre-canceled timer produced value");
    for (int i = 0; i < 100; ++i) {
        auto cancellation = lexec::inplace_stop_source{};
        auto future = std::async(std::launch::async, [&] {
            return lexec::sync_wait(
                lexec::write_env(lrclexec::schedule_after(fixture.scheduler, 1ms),
                                 lexec::prop{lexec::get_stop_token, cancellation.get_token()}));
        });
        if (i % 2 == 0)
            std::this_thread::sleep_for(1ms);
        cancellation.request_stop();
        check(future.wait_for(3s) == std::future_status::ready, "timer cancellation hung");
        (void)future.get();
    }
}

void actionTests(Fixture &fixture) {
    auto result = lexec::sync_wait(fixture.action(5));
    check(std::get<0>(*result)->sequence == std::vector<std::int32_t>{42}, "action result lost");
    for (auto const goal : {-1, -2}) {
        auto observed = std::optional<lrclexec::ActionError<Action>>{};
        auto recovered = fixture.action(goal) | lexec::let_error([&](auto error) {
                             using Error = std::decay_t<decltype(error)>;
                             if constexpr (std::is_same_v<Error, lrclexec::ActionError<Action>>)
                                 observed = error;
                             return lexec::just(std::make_shared<Action::Result>());
                         });
        (void)lexec::sync_wait(std::move(recovered));
        check(observed.has_value(), "action protocol error not delivered");
        check(observed->kind ==
                  (goal == -1 ? lrclexec::ActionErrorKind::rejected : lrclexec::ActionErrorKind::aborted),
              "wrong error kind");
        if (goal == -2)
            check(observed->result->sequence.front() == 42, "abort payload lost");
    }

    auto before = fixture.requests.load();
    auto canceled = lexec::inplace_stop_source{};
    canceled.request_stop();
    check(not lexec::sync_wait(lexec::write_env(fixture.action(1000),
                                                lexec::prop{lexec::get_stop_token, canceled.get_token()})),
          "pre-stopped action ran");
    check(fixture.requests.load() == before, "pre-stopped action sent a goal");

    fixture.delayAcceptance.store(true);
    auto source = lexec::inplace_stop_source{};
    auto terminalBefore = fixture.terminals.load();
    auto future = std::async(std::launch::async, [&] {
        return lexec::sync_wait(
            lexec::write_env(fixture.action(1000), lexec::prop{lexec::get_stop_token, source.get_token()}));
    });
    waitUntil([&] { return fixture.requests.load() > before; });
    source.request_stop();
    check(future.wait_for(10ms) == std::future_status::timeout, "completed before acceptance");
    fixture.allowAcceptance.store(true);
    check(future.wait_for(3s) == std::future_status::ready, "cancel during acceptance hung");
    check(not future.get(), "cancel during acceptance did not stop");
    check(fixture.terminals.load() > terminalBefore, "completed before remote terminal result");
    fixture.delayAcceptance.store(false);

    auto timed = fixture.action(1000) | lexec::then([](auto) noexcept { return 1; });
    auto timeout =
        lrclexec::schedule_after(fixture.scheduler, 10ms) | lexec::then([]() noexcept { return 2; });
    auto const started = std::chrono::steady_clock::now();
    auto winner = lexec::sync_wait(lexec::when_any(std::move(timed), std::move(timeout)));
    check(std::get<0>(*winner) == 2, "timeout did not win");
    check(std::chrono::steady_clock::now() - started >= fixture.cancelDelay,
          "race returned before action drain");
}

void actionBoundaryTests(Fixture &fixture) {
    fixture.acceptCancellation.store(false);
    fixture.allowTerminal.store(false);
    auto source = lexec::inplace_stop_source{};
    auto accepted = fixture.accepted.load();
    auto cancelCode = std::atomic<int>{-1};
    auto options = lrclexec::ActionOptions<Action>{};
    options.cancelResponse = [&](auto response) { cancelCode.store(response->return_code); };
    auto future = std::async(std::launch::async, [&] {
        return lexec::sync_wait(lexec::write_env(fixture.action(1, options),
                                                 lexec::prop{lexec::get_stop_token, source.get_token()}));
    });
    auto rescue = ReleaseGate{fixture.allowTerminal};
    waitUntil([&] { return fixture.accepted.load() > accepted; });
    source.request_stop();
    waitUntil([&] { return cancelCode.load() >= 0; });
    auto const waited = future.wait_for(20ms) == std::future_status::timeout;
    fixture.allowTerminal.store(true);
    check(future.wait_for(3s) == std::future_status::ready, "rejected cancellation did not finish");
    auto result = future.get();
    check(waited, "cancel response completed action before terminal");
    check(cancelCode.load() == rclcpp_action::Client<Action>::CancelResponse::ERROR_REJECTED,
          "cancel rejection was not observable");
    check(result and std::get<0>(*result)->sequence.front() == 42,
          "successful terminal was lost after rejected cancellation");
    fixture.acceptCancellation.store(true);

    // A peer that accepts cancellation but withholds its result keeps scope alive.
    fixture.allowTerminal.store(false);
    accepted = fixture.accepted.load();
    auto const cancels = fixture.cancellations.load();
    auto scope = lexec::counting_scope{};
    auto completed = std::atomic<int>{0};
    auto work = fixture.action(1000) | lexec::then([](auto) noexcept {}) |
                lexec::upon_error([](auto const &) noexcept {}) |
                lexec::upon_stopped([&]() noexcept { ++completed; });
    lexec::spawn(std::move(work), scope.get_token());
    auto joined = std::atomic<bool>{false};
    auto join = lexec::connect(scope.join(), lrclexec::detail::JoinReceiver{&joined});
    auto joinStarted = false;
    auto held = false;
    std::exception_ptr failure;
    try {
        waitUntil([&] { return fixture.accepted.load() > accepted; });
        scope.close();
        scope.request_stop();
        lexec::start(join);
        joinStarted = true;
        waitUntil([&] { return fixture.cancellations.load() > cancels; });
        (void)lexec::sync_wait(lrclexec::schedule_after(fixture.scheduler, 20ms));
        held = not joined.load() and completed.load() == 0;
    } catch (...) {
        failure = std::current_exception();
    }
    fixture.allowTerminal.store(true);
    scope.close();
    scope.request_stop();
    if (not joinStarted)
        lexec::start(join);
    try {
        waitUntil([&] { return joined.load(); });
    } catch (...) {
        std::cerr << "withheld terminal could not be rescued\n";
        std::_Exit(1);
    }
    (void)lexec::sync_wait(lexec::schedule(fixture.scheduler));
    if (failure)
        std::rethrow_exception(failure);
    check(joined.load(), "scope join notification was lost");
    check(held, "scope completed while peer withheld terminal");
    check(completed.load() == 1, "cancel did not complete exactly once");
}

struct FaultContext final : lrclexec::detail::ExecutionContext {
    explicit FaultContext(std::shared_ptr<lrclexec::detail::ExecutionContext> inner_)
        : inner{std::move(inner_)} {}
    void post(std::function<void()> task) override {
        if (posts.fetch_add(1) + 1 == failAt or failNext.exchange(false)) {
            ++failures;
            throw std::runtime_error{"injected post failure"};
        }
        inner->post(std::move(task));
        ++delivered;
    }
    rclcpp::Node &node() const noexcept override { return inner->node(); }
    rclcpp::CallbackGroup::SharedPtr callbackGroup() const noexcept override {
        return inner->callbackGroup();
    }
    std::shared_ptr<lrclexec::detail::ExecutionContext> inner;
    std::atomic<int> posts{0};
    std::atomic<int> delivered{0};
    std::atomic<int> failures{0};
    std::atomic<bool> failNext{false};
    int failAt = 2;
};

void feedbackTests(Fixture &fixture) {
    for (bool const throws : {false, true}) {
        auto context = std::make_shared<FaultContext>(fixture.scheduler.executionContext());
        context->failAt = -1;
        auto scheduler = lrclexec::TimerScheduler{context};
        fixture.allowTerminal.store(false);
        auto calls = std::atomic<int>{0};
        auto errorObserved = false;
        auto const terminals = fixture.terminals.load();
        auto options = lrclexec::ActionOptions<Action>{};
        options.feedback = [&, throws](auto feedback) {
            check(feedback->sequence.front() == 7, "feedback payload lost");
            ++calls;
            if (throws)
                throw std::runtime_error{"injected observer failure"};
        };
        auto goal = Action::Goal{};
        goal.order = throws ? 1000 : 40;
        auto future = std::async(std::launch::async, [&] {
            return lexec::sync_wait(lrclexec::execute_action(scheduler, fixture.client, goal, options) |
                                    lexec::let_error([&](auto const &) {
                                        errorObserved = true;
                                        return lexec::just(std::make_shared<Action::Result>());
                                    }));
        });
        auto rescue = ReleaseGate{fixture.allowTerminal};
        waitUntil([&] { return context->delivered.load() >= 2; });
        (void)lexec::sync_wait(lexec::schedule(fixture.scheduler));
        fixture.publishFeedback.store(true);
        waitUntil([&] { return calls.load() > 0; });
        fixture.allowTerminal.store(true);
        check(future.wait_for(3s) == std::future_status::ready, "feedback action did not drain");
        (void)future.get();
        auto const atTerminal = calls.load();
        (void)lexec::sync_wait(lrclexec::schedule_after(fixture.scheduler, 20ms));
        check(errorObserved == throws and fixture.terminals.load() > terminals,
              "observer failure or terminal result was lost");
        check(calls.load() == atTerminal, "feedback ran after action completion");
    }
}

void scopeTest() {
    auto node = std::make_shared<rclcpp::Node>("lrclexec_scope_test");
    auto scheduler = lrclexec::TimerScheduler{node};
    auto executor = rclcpp::executors::SingleThreadedExecutor{};
    executor.add_node(node);
    auto scope = lexec::counting_scope{};
    auto source = lexec::inplace_stop_source{};
    auto work =
        lrclexec::schedule_after(scheduler, 5s) | lexec::upon_error([](std::exception_ptr) noexcept {});
    lexec::spawn(std::move(work), scope.get_token());
    auto stopper = std::thread{[&] {
        std::this_thread::sleep_for(20ms);
        source.request_stop();
    }};
    lrclexec::spin_with_scope(executor, scope, source.get_token());
    stopper.join();
    check(lexec::sync_wait(scope.join()).has_value(), "scope did not join");
    check(not scope.get_token().try_associate(), "closed scope accepted more work");
}

void feedbackTerminalRaceTest(Fixture &fixture) {
    auto context = std::make_shared<FaultContext>(fixture.scheduler.executionContext());
    context->failAt = -1;
    auto scheduler = lrclexec::TimerScheduler{context};
    fixture.publishFeedback.store(false);
    fixture.allowTerminal.store(false);
    auto release = std::atomic<bool>{false};
    auto entered = std::atomic<bool>{false};
    auto observedError = false;
    auto options = lrclexec::ActionOptions<Action>{};
    options.feedback = [&](auto) {
        fixture.publishFeedback.store(false);
        // The fixture publishes exactly one feedback message per armed test.
        if (entered.exchange(true))
            return;
        while (not release.load())
            std::this_thread::sleep_for(1ms);
    };
    auto goal = Action::Goal{};
    goal.order = 1;
    auto future = std::async(std::launch::async, [&] {
        return lexec::sync_wait(lrclexec::execute_action(scheduler, fixture.client, goal, options) |
                                lexec::let_error([&](auto const &) {
                                    observedError = true;
                                    return lexec::just(std::make_shared<Action::Result>());
                                }));
    });
    auto rescue = ReleaseGate{release};
    auto terminalRescue = ReleaseGate{fixture.allowTerminal};
    waitUntil([&] { return context->delivered.load() >= 2; });
    (void)lexec::sync_wait(lexec::schedule(fixture.scheduler));
    fixture.publishFeedback.store(true);
    waitUntil([&] { return entered.load(); });
    // The only feedback callback is now blocked; the next SDK post is the result.
    context->failNext.store(true);
    fixture.allowTerminal.store(true);
    waitUntil([&] { return context->failures.load() > 0; });
    auto const waited = future.wait_for(20ms) == std::future_status::timeout;
    release.store(true);
    check(future.wait_for(3s) == std::future_status::ready, "feedback terminal race did not drain");
    (void)future.get();
    check(waited and observedError, "terminal completed before observer returned");
}

void postFailureTests(Fixture &fixture) {
    {
        auto context = std::make_shared<FaultContext>(fixture.scheduler.executionContext());
        auto source = lexec::inplace_stop_source{};
        auto observed = false;
        auto future = std::async(std::launch::async, [&] {
            return lexec::sync_wait(
                lexec::write_env(lrclexec::schedule_after(lrclexec::TimerScheduler{context}, 5s),
                                 lexec::prop{lexec::get_stop_token, source.get_token()}) |
                lexec::upon_error([&](std::exception_ptr) noexcept { observed = true; }));
        });
        waitUntil([&] { return context->posts.load() == 1; });
        // Let registration run before injecting failure of the stop delivery.
        (void)lexec::sync_wait(lexec::schedule(fixture.scheduler));
        source.request_stop();
        check(future.wait_for(500ms) == std::future_status::ready, "failed timer stop waited for expiry");
        (void)future.get();
        check(observed, "timer stop post failure was not reported");
    }
    // Failure delivering acceptance must still cancel and observe the terminal result.
    // Failure delivering the terminal result must not lose the receiver.
    for (int const failAt : {2, 3}) {
        auto context = std::make_shared<FaultContext>(fixture.scheduler.executionContext());
        context->failAt = failAt;
        auto scheduler = lrclexec::TimerScheduler{context};
        auto goal = Action::Goal{};
        goal.order = failAt == 2 ? 1000 : 5;
        auto observed = false;
        auto const terminals = fixture.terminals.load();
        auto work = lrclexec::execute_action(scheduler, fixture.client, std::move(goal)) |
                    lexec::let_error([&](auto const &) {
                        observed = true;
                        return lexec::just(std::make_shared<Action::Result>());
                    });
        (void)lexec::sync_wait(std::move(work));
        check(observed, "post failure was not reported");
        check(fixture.terminals.load() > terminals, "post failure completed before remote drain");
    }
    // Completing a rejected goal from the SDK callback stack would deadlock
    // this recovery's direct async_send_goal on the SDK goal-request mutex.
    auto context = std::make_shared<FaultContext>(fixture.scheduler.executionContext());
    auto scheduler = lrclexec::TimerScheduler{context};
    auto rejected = Action::Goal{};
    rejected.order = -1;
    std::shared_future<rclcpp_action::ClientGoalHandle<Action>::SharedPtr> native;
    auto recovery =
        lrclexec::execute_action(scheduler, fixture.client, rejected) | lexec::let_error([&](auto const &) {
            auto goal = Action::Goal{};
            goal.order = 5;
            native = fixture.client->async_send_goal(goal);
            return lexec::just(std::make_shared<Action::Result>());
        });
    (void)lexec::sync_wait(std::move(recovery));
    check(native.valid() and native.wait_for(3s) == std::future_status::ready,
          "queue-failure recovery could not reenter ROS client");
    auto result = fixture.client->async_get_result(native.get());
    check(result.wait_for(3s) == std::future_status::ready and
              result.get().code == rclcpp_action::ResultCode::SUCCEEDED,
          "reentrant recovery goal did not finish");
}

void drainExceptionTest() {
    auto node = std::make_shared<rclcpp::Node>("lrclexec_drain_exception_test");
    auto scheduler = lrclexec::TimerScheduler{node};
    auto scope = lexec::counting_scope{};
    auto stop = lexec::inplace_stop_source{};
    auto executor = rclcpp::executors::SingleThreadedExecutor{};
    executor.add_node(node);
    auto timer = rclcpp::TimerBase::SharedPtr{};
    timer = node->create_wall_timer(0ms, [&] {
        timer->cancel();
        throw std::runtime_error{"injected drain callback failure"};
    });
    lexec::spawn(lrclexec::schedule_after(scheduler, 5s) |
                     lexec::upon_error([](std::exception_ptr) noexcept {}),
                 scope.get_token());
    stop.request_stop();
    auto observed = false;
    try {
        lrclexec::spin_with_scope(executor, scope, stop.get_token());
    } catch (std::runtime_error const &) {
        observed = true;
    }
    check(observed, "drain exception was not propagated");
    (void)lexec::sync_wait(scope.join());
}

void queueExceptionTest() {
    auto node = std::make_shared<rclcpp::Node>("lrclexec_queue_exception_test");
    auto scheduler = lrclexec::TimerScheduler{node};
    auto executor = rclcpp::executors::SingleThreadedExecutor{};
    executor.add_node(node);
    auto secondRan = false;
    auto thirdRan = false;
    auto observed = false;
    scheduler.executionContext()->post([] { throw std::runtime_error{"injected queue failure"}; });
    scheduler.executionContext()->post([&] { secondRan = true; });
    auto const deadline = std::chrono::steady_clock::now() + 3s;
    while (not observed) {
        check(std::chrono::steady_clock::now() < deadline, "queue exception was not delivered");
        try {
            executor.spin_once(10ms);
        } catch (std::runtime_error const &) {
            observed = true;
        }
    }
    check(secondRan, "queue exception abandoned the rest of its batch");
    scheduler.executionContext()->post([&] { thirdRan = true; });
    while (not thirdRan) {
        check(std::chrono::steady_clock::now() < deadline, "queue group did not recover after exception");
        executor.spin_once(10ms);
    }
}

void serverTests(Fixture &fixture) {
    auto scope = lexec::counting_scope{};
    auto running = std::atomic<int>{0};
    auto maximum = std::atomic<int>{0};
    auto server = lrclexec::make_action_server_preempt<Action>(
        fixture.scheduler, scope, "lrclexec_adapted_server", [&](auto handle) {
            auto const count = running.fetch_add(1) + 1;
            if (count > maximum.load())
                maximum.store(count);
            auto result = std::make_shared<Action::Result>();
            result->sequence = {handle->get_goal()->order};
            return lrclexec::schedule_after(fixture.scheduler,
                                            std::chrono::milliseconds{handle->get_goal()->order}) |
                   lexec::then([&, result]() noexcept {
                       running.fetch_sub(1);
                       return result;
                   }) |
                   lexec::let_stopped([&]() noexcept {
                       running.fetch_sub(1);
                       return lexec::just_stopped();
                   });
        });
    auto client = rclcpp_action::create_client<Action>(fixture.node, "lrclexec_adapted_server");
    // Join before either native Action waitable can lose its last owner.
    auto stopExecutor = StopExecutor{fixture};
    check(client->wait_for_action_server(3s), "adapted server discovery failed");
    auto action = [&](int const delay) {
        auto goal = Action::Goal{};
        goal.order = delay;
        return lrclexec::execute_action(fixture.scheduler, client, std::move(goal));
    };
    auto first = std::async(std::launch::async, [&] {
        return lexec::sync_wait(action(1000) | lexec::let_error([](auto) {
                                    return lexec::just(std::make_shared<Action::Result>());
                                }));
    });
    waitUntil([&] { return running.load() == 1; });
    auto second = lexec::sync_wait(action(10));
    check(std::get<0>(*second)->sequence.front() == 10, "replacement goal failed");
    check(first.wait_for(3s) == std::future_status::ready, "preempted goal did not finish");
    check(std::get<0>(*first.get())->sequence.empty(), "preempted goal succeeded");
    check(maximum.load() == 1, "replacement began before previous task drained");

    auto source = lexec::inplace_stop_source{};
    auto canceled = std::async(std::launch::async, [&] {
        return lexec::sync_wait(
            lexec::write_env(action(1000), lexec::prop{lexec::get_stop_token, source.get_token()}));
    });
    waitUntil([&] { return running.load() == 1; });
    source.request_stop();
    check(canceled.wait_for(3s) == std::future_status::ready, "server client cancellation hung");
    check(not canceled.get(), "server client cancellation did not map to canceled");
    server.close();
    scope.close();
    scope.request_stop();
    (void)lexec::sync_wait(scope.join());
    check(running.load() == 0, "server left work running");
}

struct ExclusiveResource {
    explicit ExclusiveResource(std::atomic<int> &live_) : live{live_} { live.fetch_add(1); }
    ~ExclusiveResource() { live.fetch_sub(1); }
    std::atomic<int> &live;
};

void pendingServerTests(Fixture &fixture) {
    auto scope = lexec::counting_scope{};
    auto live = std::atomic<int>{0};
    auto overlaps = std::atomic<int>{0};
    auto factories = std::atomic<int>{0};
    auto cleaning = std::atomic<int>{0};
    auto server = lrclexec::make_action_server_preempt<Action>(
        fixture.scheduler, scope, "lrclexec_pending_server", [&](auto handle) {
            if (handle->get_goal()->order == -99)
                throw std::runtime_error{"injected factory failure"};
            if (live.load() != 0)
                overlaps.fetch_add(1);
            factories.fetch_add(1);
            auto resource = std::make_shared<ExclusiveResource>(live);
            auto result = std::make_shared<Action::Result>();
            result->sequence = {handle->get_goal()->order};
            return lrclexec::schedule_after(fixture.scheduler,
                                            std::chrono::milliseconds{handle->get_goal()->order}) |
                   lexec::then([resource, result]() noexcept { return result; }) |
                   lexec::let_stopped([&, resource] {
                       cleaning.fetch_add(1);
                       auto cleanup =
                           lrclexec::schedule_after(fixture.scheduler, 100ms) |
                           lexec::let_value([resource]() noexcept { return lexec::just_stopped(); });
                       return lexec::write_env(std::move(cleanup),
                                               lexec::prop{lexec::get_stop_token, lexec::never_stop_token{}});
                   });
        });
    auto client = rclcpp_action::create_client<Action>(fixture.node, "lrclexec_pending_server");
    auto stopExecutor = StopExecutor{fixture};
    check(client->wait_for_action_server(3s), "pending server discovery failed");
    auto send = [&](int const delay) {
        auto goal = Action::Goal{};
        goal.order = delay;
        auto future = client->async_send_goal(goal);
        check(future.wait_for(3s) == std::future_status::ready, "goal acceptance timed out");
        auto handle = future.get();
        check(static_cast<bool>(handle), "pending server rejected goal");
        return handle;
    };
    auto result = [&](auto handle) {
        auto future = client->async_get_result(handle);
        check(future.wait_for(3s) == std::future_status::ready, "pending result timed out");
        return future.get().code;
    };

    check(result(send(-99)) == rclcpp_action::ResultCode::ABORTED, "factory exception did not abort goal");
    auto first = send(1000);
    waitUntil([&] { return live.load() == 1; });
    auto second = send(20);
    waitUntil([&] { return cleaning.load() == 1; });
    auto third = send(5);
    check(result(second) == rclcpp_action::ResultCode::ABORTED, "superseded pending goal was not aborted");
    check(result(first) == rclcpp_action::ResultCode::ABORTED, "active preemption was not aborted");
    check(result(third) == rclcpp_action::ResultCode::SUCCEEDED, "latest pending goal did not succeed");
    waitUntil([&] { return live.load() == 0; });
    check(factories.load() == 2, "superseded pending factory ran");
    check(overlaps.load() == 0, "new factory ran before old operation released its resource");

    first = send(1000);
    waitUntil([&] { return live.load() == 1; });
    second = send(20);
    waitUntil([&] { return cleaning.load() == 2; });
    auto cancel = client->async_cancel_goal(second);
    check(cancel.wait_for(3s) == std::future_status::ready, "pending cancel request timed out");
    check(cancel.get()->return_code == 0, "pending cancellation was rejected");
    check(result(second) == rclcpp_action::ResultCode::CANCELED, "pending cancellation was not terminal");
    check(result(first) == rclcpp_action::ResultCode::ABORTED,
          "old goal did not drain after pending cancellation");
    waitUntil([&] { return live.load() == 0; });
    check(factories.load() == 3, "canceled pending factory ran");

    first = send(1000);
    waitUntil([&] { return live.load() == 1; });
    second = send(20);
    waitUntil([&] { return cleaning.load() == 3; });
    server.close();
    scope.close();
    scope.request_stop();
    (void)lexec::sync_wait(scope.join());
    check(result(first) == rclcpp_action::ResultCode::ABORTED, "close did not drain active goal");
    check(result(second) == rclcpp_action::ResultCode::ABORTED, "close did not abort pending goal");
    check(factories.load() == 4 and live.load() == 0, "scope joined before server resources were released");
}

void emptyServerCloseTest() {
    auto node = std::make_shared<rclcpp::Node>("lrclexec_empty_close_test");
    auto scheduler = lrclexec::TimerScheduler{node};
    auto scope = lexec::counting_scope{};
    auto server =
        lrclexec::make_action_server_preempt<Action>(scheduler, scope, "lrclexec_empty_server", [](auto) {
            return lexec::just(std::make_shared<Action::Result>());
        });
    auto executor = rclcpp::executors::SingleThreadedExecutor{};
    executor.add_node(node);
    server.close();
    scope.close();
    auto joined = std::atomic<bool>{false};
    auto join = lexec::connect(scope.join(), lrclexec::detail::JoinReceiver{&joined});
    lexec::start(join);
    check(not joined.load(), "empty server close was not tracked by scope");
    auto stop = lexec::inplace_stop_source{};
    stop.request_stop();
    lrclexec::spin_with_scope(executor, scope, stop.get_token());
    check(joined.load(), "empty server close did not drain");
}

int main(int argc, char **argv) {
    rclcpp::init(argc, argv, rclcpp::InitOptions{}, rclcpp::SignalHandlerOptions::None);
    try {
        {
            std::cerr << "creating fixture\n";
            auto fixture = Fixture{};
            std::cerr << "timer checks\n";
            timerTests(fixture);
            std::cerr << "action checks\n";
            actionTests(fixture);
            actionBoundaryTests(fixture);
            feedbackTests(fixture);
            feedbackTerminalRaceTest(fixture);
            std::cerr << "post failure checks\n";
            postFailureTests(fixture);
            std::cerr << "server checks\n";
            serverTests(fixture);
            fixture.startExecutor();
            pendingServerTests(fixture);
        }
        std::cerr << "scope checks\n";
        scopeTest();
        drainExceptionTest();
        queueExceptionTest();
        emptyServerCloseTest();
        rclcpp::shutdown();
        std::cout << "timer, action cancellation, timeout drain, and scope checks passed\n";
        return 0;
    } catch (std::exception const &error) {
        std::cerr << error.what() << '\n';
        rclcpp::shutdown();
        return 1;
    }
}
