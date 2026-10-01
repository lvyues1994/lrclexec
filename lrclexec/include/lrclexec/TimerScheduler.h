#pragma once

#include <chrono>
#include <lexec/execution.hpp>
#include <lrclexec/Completion.h>
#include <lrclexec/ExecutionContext.h>
#include <memory>
#include <optional>
#include <rclcpp/create_timer.hpp>
#include <utility>

namespace lrclexec {
struct TimerScheduler;
enum struct TimerClock { steady, node };

namespace detail {

template <class Receiver> struct TimerState final : std::enable_shared_from_this<TimerState<Receiver>> {
    using Token = lexec::stop_token_of_t<lexec::env_of_t<Receiver>>;
    struct Stop {
        void operator()() const noexcept {
            if (auto state = weak.lock()) {
                try {
                    state->context->post([weak = weak] {
                        if (auto current = weak.lock())
                            current->finish(lexec::set_stopped);
                    });
                } catch (...) {
                    state->failedPost(std::current_exception());
                }
            }
        }
        std::weak_ptr<TimerState> weak;
    };
    using StopCallback = lexec::stop_callback_for_t<Token, Stop>;

    TimerState(std::shared_ptr<ExecutionContext> context_, std::chrono::nanoseconds const delay_,
               rclcpp::Clock::SharedPtr clock_, Receiver receiver)
        : context{std::move(context_)}, delay{delay_}, clock{std::move(clock_)},
          token{lexec::get_stop_token(lexec::get_env(receiver))}, completion{std::move(receiver)} {}

    void start() noexcept {
        auto state = this->shared_from_this();
        try {
            context->post([state] { state->begin(); });
        } catch (...) {
            finish(lexec::set_error, std::current_exception());
        }
    }
    void begin() noexcept {
        std::exception_ptr error;
        auto stopped = false;
        {
            auto const lock = std::lock_guard<std::recursive_mutex>{mutex};
            if (completion.isComplete())
                return;
            try {
                stopCallback.emplace(token, Stop{this->weak_from_this()});
                stopped = token.stop_requested();
                error = completion.pendingError();
                if (not stopped and not error and delay.count() > 0)
                    arm();
            } catch (...) {
                error = std::current_exception();
            }
            initialized = true;
        }
        if (error)
            finish(lexec::set_error, error);
        else if (stopped)
            finish(lexec::set_stopped);
        else if (delay.count() <= 0)
            finish(lexec::set_value);
    }

  private:
    void arm() {
        auto const weak = this->weak_from_this();
        auto &node = context->node();
        auto callback = [weak] {
            if (auto state = weak.lock())
                state->expired();
        };
        if (clock)
            timer = rclcpp::create_timer(clock, delay, callback, context->callbackGroup(),
                                         node.get_node_base_interface().get(),
                                         node.get_node_timers_interface().get(), false);
        else
            timer = rclcpp::create_wall_timer(delay, callback, context->callbackGroup(),
                                              node.get_node_base_interface().get(),
                                              node.get_node_timers_interface().get(), false);
        timer->reset();
    }
    void expired() noexcept {
        std::exception_ptr error;
        auto stopped = false;
        {
            auto const lock = std::lock_guard<std::recursive_mutex>{mutex};
            if (completion.isComplete())
                return;
            error = completion.pendingError();
            stopped = token.stop_requested();
        }
        if (error)
            finish(lexec::set_error, error);
        else if (stopped)
            finish(lexec::set_stopped);
        else
            finish(lexec::set_value);
    }
    void failedPost(std::exception_ptr error) noexcept {
        completion.rememberError(error);
        {
            auto const lock = std::lock_guard<std::recursive_mutex>{mutex};
            // A callback may run synchronously while its registration is constructing.
            if (not initialized)
                return;
        }
        finish(lexec::set_error, error);
    }
    template <class Tag, class... Values> void finish(Tag tag, Values &&...values) noexcept {
        std::unique_ptr<Receiver> receiver;
        rclcpp::TimerBase::SharedPtr timerToCancel;
        {
            auto const lock = std::lock_guard<std::recursive_mutex>{mutex};
            receiver = completion.takeReceiver();
            if (not receiver)
                return;
            timerToCancel = std::exchange(timer, {});
        }
        stopCallback.reset();
        if (timerToCancel) {
            try {
                timerToCancel->cancel();
            } catch (...) {
                lexec::set_error(std::move(*receiver), std::current_exception());
                return;
            }
        }
        tag(std::move(*receiver), std::forward<Values>(values)...);
    }

    std::shared_ptr<ExecutionContext> context;
    std::chrono::nanoseconds delay;
    rclcpp::Clock::SharedPtr clock;
    Token token;
    Completion<Receiver> completion;
    std::optional<StopCallback> stopCallback;
    rclcpp::TimerBase::SharedPtr timer;
    std::recursive_mutex mutex;
    bool initialized = false;
};

template <class State> struct SharedOperation {
    using operation_state_concept = lexec::operation_state_t;
    explicit SharedOperation(std::shared_ptr<State> state_) : state{std::move(state_)} {}
    SharedOperation(SharedOperation &&) = delete;
    void start() & noexcept {
        auto current = state;
        current->start();
    }

  private:
    std::shared_ptr<State> state;
};

struct TimerAttributes {
    TimerScheduler query(lexec::get_completion_scheduler_t<lexec::set_value_t>) const noexcept;
    std::shared_ptr<ExecutionContext> context;
    rclcpp::Clock::SharedPtr clock;
};

} // namespace detail

struct TimerSender {
    using sender_concept = lexec::sender_t;
    using completion_signatures =
        lexec::completion_signatures<lexec::set_value_t(), lexec::set_error_t(std::exception_ptr),
                                     lexec::set_stopped_t()>;

    template <class Receiver> auto connect(Receiver receiver) const {
        using State = detail::TimerState<Receiver>;
        return detail::SharedOperation<State>{
            std::make_shared<State>(context, delay, clock, std::move(receiver))};
    }
    detail::TimerAttributes get_env() const noexcept { return {context, clock}; }

    std::shared_ptr<detail::ExecutionContext> context;
    std::chrono::nanoseconds delay;
    rclcpp::Clock::SharedPtr clock;
};

// Copies share one executor queue and one mutually exclusive callback group.
struct TimerScheduler {
    using scheduler_concept = lexec::scheduler_t;
    explicit TimerScheduler(std::shared_ptr<rclcpp::Node> node, TimerClock timerClock = TimerClock::steady)
        : context{detail::makeExecutionContext(std::move(node))},
          clock{timerClock == TimerClock::node ? context->node().get_clock() : nullptr} {}
    explicit TimerScheduler(std::shared_ptr<detail::ExecutionContext> context_,
                            rclcpp::Clock::SharedPtr clock_ = {})
        : context{std::move(context_)}, clock{std::move(clock_)} {}
    TimerSender schedule() const noexcept { return {context, std::chrono::nanoseconds{0}, clock}; }
    TimerSender schedule_after(std::chrono::nanoseconds const delay) const noexcept {
        return {context, delay, clock};
    }
    rclcpp::Node &node() const noexcept { return context->node(); }
    rclcpp::CallbackGroup::SharedPtr callbackGroup() const noexcept { return context->callbackGroup(); }
    std::shared_ptr<detail::ExecutionContext> executionContext() const noexcept { return context; }
    friend bool operator==(TimerScheduler const &a, TimerScheduler const &b) noexcept {
        return a.context == b.context and a.clock == b.clock;
    }
    friend bool operator!=(TimerScheduler const &a, TimerScheduler const &b) noexcept { return not(a == b); }

  private:
    std::shared_ptr<detail::ExecutionContext> context;
    rclcpp::Clock::SharedPtr clock;
};

inline TimerScheduler
detail::TimerAttributes::query(lexec::get_completion_scheduler_t<lexec::set_value_t>) const noexcept {
    return TimerScheduler{context, clock};
}

inline auto schedule_after(TimerScheduler const &scheduler, std::chrono::nanoseconds const delay) noexcept {
    return scheduler.schedule_after(delay);
}

} // namespace lrclexec
