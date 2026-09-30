#pragma once

#include <cstdint>
#include <lrclexec/TimerScheduler.h>
#include <rclcpp_action/client.hpp>
#include <stdexcept>

namespace lrclexec {

enum struct ActionErrorKind : std::uint8_t { rejected, aborted, unknownResult };

template <class Action> struct ActionError {
    ActionErrorKind kind;
    typename Action::Result::SharedPtr result;
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
                    state->cancel();
                }
            }
        }
        std::weak_ptr<ActionState> weak;
    };
    using StopCallback = lexec::stop_callback_for_t<Token, Stop>;

    ActionState(std::shared_ptr<ExecutionContext> context_, std::shared_ptr<Client> client_,
                typename Action::Goal goal_, Receiver receiver)
        : context{std::move(context_)}, client{std::move(client_)}, goal{std::move(goal_)},
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
                if (auto state = weak.lock()) {
                    state->dispatch([state, handle = std::move(handle)] { state->accepted(handle); });
                }
            };
            options.result_callback = [weak](typename Handle::WrappedResult const &result) {
                if (auto state = weak.lock()) {
                    state->dispatch([state, result] { state->resultReady(result); });
                }
            };
            (void)client->async_send_goal(goal, options);
        } catch (...) {
            finish(lexec::set_error, std::current_exception());
        }
    }

  private:
    template <class Function> void dispatch(Function function) noexcept {
        try {
            context->post(function);
        } catch (...) {
            completion.rememberError(std::current_exception());
            // Retain the remote handle and terminal result even if posting fails.
            function();
        }
    }
    void accepted(typename Handle::SharedPtr const &handle_) noexcept {
        auto const lock = std::lock_guard<std::recursive_mutex>{lifecycleMutex};
        if (completion.isComplete())
            return;
        if (not handle_) {
            finish(lexec::set_error, ActionError<Action>{ActionErrorKind::rejected, {}});
            return;
        }
        handle = handle_;
        try {
            stopCallback.emplace(token, Stop{this->weak_from_this()});
            if (token.stop_requested() or completion.pendingError())
                cancel();
        } catch (...) {
            completion.rememberError(std::current_exception());
            cancel();
        }
    }
    void cancel() noexcept {
        auto const lock = std::lock_guard<std::recursive_mutex>{lifecycleMutex};
        if (completion.isComplete() or not handle or cancelSent)
            return;
        cancelSent = true;
        try {
            (void)client->async_cancel_goal(handle);
        } catch (rclcpp_action::exceptions::UnknownGoalHandleError const &) {
            // The terminal response may already have removed the handle.
        } catch (...) {
            completion.rememberError(std::current_exception());
        }
    }
    void resultReady(typename Handle::WrappedResult const &result) noexcept {
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
        // Completion excludes future registration; never wait for a running stop
        // callback while holding the mutex that its fallback cancel() needs.
        stopCallback.reset();
        tag(std::move(*receiver), std::forward<Values>(values)...);
    }

    std::shared_ptr<ExecutionContext> context;
    std::shared_ptr<Client> client;
    typename Action::Goal goal;
    Token token;
    Completion<Receiver> completion;
    typename Handle::SharedPtr handle;
    std::optional<StopCallback> stopCallback;
    bool cancelSent = false;
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
            std::make_shared<State>(context, client, goal, std::move(receiver))};
    }
    template <class Receiver> auto connect(Receiver receiver) && {
        using State = detail::ActionState<Action, Receiver>;
        return detail::SharedOperation<State>{
            std::make_shared<State>(context, client, std::move(goal), std::move(receiver))};
    }

    std::shared_ptr<detail::ExecutionContext> context;
    std::shared_ptr<rclcpp_action::Client<Action>> client;
    typename Action::Goal goal;
};

template <class Action>
auto execute_action(TimerScheduler const &scheduler, std::shared_ptr<rclcpp_action::Client<Action>> client,
                    typename Action::Goal goal) {
    if (not client)
        throw std::invalid_argument{"lrclexec: execute_action needs a client"};
    return ActionSender<Action>{scheduler.executionContext(), std::move(client), std::move(goal)};
}

} // namespace lrclexec
