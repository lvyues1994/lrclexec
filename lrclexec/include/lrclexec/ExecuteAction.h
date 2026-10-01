#pragma once

#include <cstdint>
#include <functional>
#include <lrclexec/TimerScheduler.h>
#include <rclcpp_action/client.hpp>
#include <stdexcept>

namespace lrclexec {

enum struct ActionErrorKind : std::uint8_t { rejected, aborted, unknownResult };

template <class Action> struct ActionError {
    ActionErrorKind kind;
    typename Action::Result::SharedPtr result;
};

template <class Action> struct ActionOptions {
    std::function<void(std::shared_ptr<typename Action::Feedback const>)> feedback;
    std::function<void(typename rclcpp_action::Client<Action>::CancelResponse::SharedPtr)> cancelResponse;
};

namespace detail {

template <class Action, class Receiver>
struct ActionState final : std::enable_shared_from_this<ActionState<Action, Receiver>> {
    using Client = rclcpp_action::Client<Action>;
    using Handle = rclcpp_action::ClientGoalHandle<Action>;
    using Token = lexec::stop_token_of_t<lexec::env_of_t<Receiver>>;
    struct Stop {
        void operator()() const noexcept {
            if (auto state = weak.lock()) {
                try {
                    state->context->post([weak = weak] {
                        if (auto current = weak.lock())
                            current->cancel();
                    });
                } catch (...) {
                    state->completion.rememberError(std::current_exception());
                    // Stop callback teardown can wait for this call. Never
                    // enter SDK locks from this stack, even when posting fails.
                    try {
                        state->context->post([weak = weak] {
                            if (auto current = weak.lock())
                                current->cancel();
                        });
                    } catch (...) {
                        // Keep waiting for a remote terminal result.
                    }
                }
            }
        }
        std::weak_ptr<ActionState> weak;
    };
    using StopCallback = lexec::stop_callback_for_t<Token, Stop>;

    ActionState(std::shared_ptr<ExecutionContext> context_, std::shared_ptr<Client> client_,
                typename Action::Goal goal_, ActionOptions<Action> options_, Receiver receiver)
        : context{std::move(context_)}, client{std::move(client_)}, goal{std::move(goal_)},
          observers{std::move(options_)}, token{lexec::get_stop_token(lexec::get_env(receiver))},
          completion{std::move(receiver)} {}

    void start() noexcept {
        auto state = this->shared_from_this();
        try {
            context->post([state] { state->begin(); });
        } catch (...) {
            finish(lexec::set_error, std::current_exception());
        }
    }
    void begin() noexcept {
        if (completion.isComplete())
            return;
        if (token.stop_requested()) {
            finish(lexec::set_stopped);
            return;
        }
        try {
            auto const weak = this->weak_from_this();
            typename Client::SendGoalOptions options;
            options.goal_response_callback = [weak](typename Handle::SharedPtr handle) {
                if (auto state = weak.lock())
                    state->queueAcceptance(std::move(handle));
            };
            options.result_callback = [weak](typename Handle::WrappedResult const &result) {
                if (auto state = weak.lock())
                    state->queueResult(result);
            };
            if (observers.feedback) {
                options.feedback_callback =
                    [weak](typename Handle::SharedPtr,
                           std::shared_ptr<typename Action::Feedback const> feedback) {
                        if (auto state = weak.lock()) {
                            state->postObserver([state, feedback = std::move(feedback)] {
                                state->observers.feedback(feedback);
                            });
                        }
                    };
            }
            (void)client->async_send_goal(goal, options);
        } catch (...) {
            finish(lexec::set_error, std::current_exception());
        }
    }

  private:
    void queueAcceptance(typename Handle::SharedPtr handle_) noexcept {
        {
            auto const lock = std::lock_guard<std::recursive_mutex>{lifecycleMutex};
            pendingAcceptance = std::move(handle_);
        }
        requestDrain();
    }
    void queueResult(typename Handle::WrappedResult const &result) noexcept {
        {
            auto const lock = std::lock_guard<std::recursive_mutex>{lifecycleMutex};
            pendingResult = result;
        }
        requestDrain();
    }
    void requestDrain() noexcept {
        // SDK callbacks can hold handle/map locks. Even a failed queue post
        // must not run receiver completion (and arbitrary user code) inline.
        for (int attempt = 0; attempt < 2; ++attempt) {
            try {
                context->post([state = this->shared_from_this()] { state->drain(); });
                return;
            } catch (...) {
                completion.rememberError(std::current_exception());
            }
        }
        // Retain both responses for a later executor entry. Permanent queue
        // failure cannot guarantee progress, and must not free in-flight work.
    }
    void drain() noexcept {
        std::optional<typename Handle::SharedPtr> acceptance;
        std::optional<typename Handle::WrappedResult> result;
        {
            auto const lock = std::lock_guard<std::recursive_mutex>{lifecycleMutex};
            acceptance = std::exchange(pendingAcceptance, {});
            result = std::exchange(pendingResult, {});
        }
        if (acceptance)
            accepted(*acceptance);
        if (result)
            resultReady(*result);
    }
    void accepted(typename Handle::SharedPtr const &handle_) noexcept {
        if (not handle_) {
            if (auto error = completion.pendingError())
                finish(lexec::set_error, error);
            else
                finish(lexec::set_error, ActionError<Action>{ActionErrorKind::rejected, {}});
            return;
        }
        auto cancelNow = false;
        {
            auto const lock = std::lock_guard<std::recursive_mutex>{lifecycleMutex};
            if (completion.isComplete())
                return;
            handle = handle_;
            try {
                stopCallback.emplace(token, Stop{this->weak_from_this()});
            } catch (...) {
                completion.rememberError(std::current_exception());
            }
            initialized = true;
            cancelNow = token.stop_requested() or completion.pendingError() or cancelRequested;
        }
        if (cancelNow)
            cancel();
    }
    void cancel() noexcept {
        drain();
        typename Handle::SharedPtr current;
        {
            auto const lock = std::lock_guard<std::recursive_mutex>{lifecycleMutex};
            cancelRequested = true;
            if (not initialized or completion.isComplete() or not handle or cancelSent)
                return;
            cancelSent = true;
            current = handle;
        }
        try {
            auto const weak = this->weak_from_this();
            (void)client->async_cancel_goal(
                current, [weak](typename Client::CancelResponse::SharedPtr response) {
                    if (auto state = weak.lock()) {
                        if (state->observers.cancelResponse) {
                            state->postObserver([state, response = std::move(response)] {
                                state->observers.cancelResponse(response);
                            });
                        }
                    }
                });
        } catch (rclcpp_action::exceptions::UnknownGoalHandleError const &) {
            // The terminal response may already have removed the handle.
        } catch (...) {
            completion.rememberError(std::current_exception());
        }
    }
    void resultReady(typename Handle::WrappedResult const &result) noexcept {
        {
            auto const lock = std::lock_guard<std::recursive_mutex>{lifecycleMutex};
            if (completion.isComplete() or terminalSeen)
                return;
            terminalSeen = true;
            if (observersInFlight > 0) {
                deferredResult = result;
                return;
            }
        }
        completeResult(result);
    }
    template <class Function> void postObserver(Function function) noexcept {
        try {
            context->post([state = this->shared_from_this(), function = std::move(function)] {
                state->observe(function);
            });
        } catch (...) {
            completion.rememberError(std::current_exception());
            // SDK feedback callbacks hold SDK locks. Never run user code or
            // cancel inline here; a second queue attempt can request cleanup.
            try {
                context->post([state = this->shared_from_this()] { state->cancel(); });
            } catch (...) {
                // The terminal result still owns the completion obligation.
            }
        }
    }
    template <class Function> void observe(Function const &function) noexcept {
        drain();
        {
            auto const lock = std::lock_guard<std::recursive_mutex>{lifecycleMutex};
            if (completion.isComplete() or terminalSeen or completion.pendingError())
                return;
            ++observersInFlight;
        }
        try {
            function();
        } catch (...) {
            completion.rememberError(std::current_exception());
        }
        std::optional<typename Handle::WrappedResult> terminal;
        {
            auto const lock = std::lock_guard<std::recursive_mutex>{lifecycleMutex};
            --observersInFlight;
            if (observersInFlight == 0)
                terminal = std::exchange(deferredResult, {});
        }
        if (terminal)
            completeResult(*terminal);
        else if (completion.pendingError())
            cancel();
    }
    void completeResult(typename Handle::WrappedResult const &result) noexcept {
        if (auto error = completion.pendingError()) {
            finish(lexec::set_error, error);
            return;
        }
        switch (result.code) {
        case rclcpp_action::ResultCode::SUCCEEDED:
            finish(lexec::set_value, result.result);
            return;
        case rclcpp_action::ResultCode::CANCELED:
            finish(lexec::set_stopped);
            return;
        case rclcpp_action::ResultCode::ABORTED:
            finish(lexec::set_error, ActionError<Action>{ActionErrorKind::aborted, result.result});
            return;
        default:
            finish(lexec::set_error, ActionError<Action>{ActionErrorKind::unknownResult, result.result});
        }
    }
    template <class Tag, class... Values> void finish(Tag tag, Values &&...values) noexcept {
        auto receiver = std::unique_ptr<Receiver>{};
        {
            auto const lock = std::lock_guard<std::recursive_mutex>{lifecycleMutex};
            receiver = completion.takeReceiver();
            if (not receiver)
                return;
        }
        // Completion excludes future registration; reset outside the state
        // mutex so teardown never waits for a stop callback while holding it.
        stopCallback.reset();
        tag(std::move(*receiver), std::forward<Values>(values)...);
    }

    std::shared_ptr<ExecutionContext> context;
    std::shared_ptr<Client> client;
    typename Action::Goal goal;
    ActionOptions<Action> observers;
    Token token;
    Completion<Receiver> completion;
    typename Handle::SharedPtr handle;
    std::optional<StopCallback> stopCallback;
    bool cancelSent = false;
    bool cancelRequested = false;
    bool initialized = false;
    bool terminalSeen = false;
    std::size_t observersInFlight = 0;
    std::optional<typename Handle::WrappedResult> deferredResult;
    std::optional<typename Handle::SharedPtr> pendingAcceptance;
    std::optional<typename Handle::WrappedResult> pendingResult;
    std::recursive_mutex lifecycleMutex;
};

} // namespace detail

template <class Action> struct ActionSender {
    using sender_concept = lexec::sender_t;
    using completion_signatures =
        lexec::completion_signatures<lexec::set_value_t(typename Action::Result::SharedPtr),
                                     lexec::set_error_t(ActionError<Action>),
                                     lexec::set_error_t(std::exception_ptr), lexec::set_stopped_t()>;

    template <class Receiver> auto connect(Receiver receiver) const & {
        using State = detail::ActionState<Action, Receiver>;
        return detail::SharedOperation<State>{
            std::make_shared<State>(context, client, goal, options, std::move(receiver))};
    }
    template <class Receiver> auto connect(Receiver receiver) && {
        using State = detail::ActionState<Action, Receiver>;
        return detail::SharedOperation<State>{std::make_shared<State>(
            context, client, std::move(goal), std::move(options), std::move(receiver))};
    }

    std::shared_ptr<detail::ExecutionContext> context;
    std::shared_ptr<rclcpp_action::Client<Action>> client;
    typename Action::Goal goal;
    ActionOptions<Action> options;
};

template <class Action>
auto execute_action(TimerScheduler const &scheduler, std::shared_ptr<rclcpp_action::Client<Action>> client,
                    typename Action::Goal goal, ActionOptions<Action> options = {}) {
    if (not client)
        throw std::invalid_argument{"lrclexec: execute_action needs a client"};
    return ActionSender<Action>{scheduler.executionContext(), std::move(client), std::move(goal),
                                std::move(options)};
}

} // namespace lrclexec
