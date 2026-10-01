#pragma once

#include <lrclexec/TimerScheduler.h>
#include <rclcpp/create_subscription.hpp>
#include <string>

namespace lrclexec {
namespace detail {

template <class Message, class Receiver>
struct TopicState final : std::enable_shared_from_this<TopicState<Message, Receiver>> {
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
        std::weak_ptr<TopicState> weak;
    };
    using StopCallback = lexec::stop_callback_for_t<Token, Stop>;

    TopicState(std::shared_ptr<ExecutionContext> context_, std::string topic_, rclcpp::QoS qos_,
               Receiver receiver)
        : context{std::move(context_)}, topic{std::move(topic_)}, qos{std::move(qos_)},
          token{lexec::get_stop_token(lexec::get_env(receiver))}, completion{std::move(receiver)} {}

    void start() noexcept {
        try {
            context->post([state = this->shared_from_this()] { state->begin(); });
        } catch (...) {
            finish(lexec::set_error, std::current_exception());
        }
    }

  private:
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
                if (not stopped and not error) {
                    auto options = rclcpp::SubscriptionOptions{};
                    options.callback_group = context->callbackGroup();
                    subscription = rclcpp::create_subscription<Message>(
                        context->node(), topic, qos,
                        [weak = this->weak_from_this()](Message const &message) {
                            if (auto state = weak.lock())
                                state->messageReady(message);
                        },
                        options);
                }
            } catch (...) {
                error = std::current_exception();
            }
            initialized = true;
        }
        if (error)
            finish(lexec::set_error, error);
        else if (stopped)
            finish(lexec::set_stopped);
    }
    void messageReady(Message const &message) noexcept {
        {
            auto const lock = std::lock_guard<std::recursive_mutex>{mutex};
            if (completion.isComplete() or messageClaimed)
                return;
            messageClaimed = true;
        }
        try {
            // A DDS loan may expire when the ROS callback returns. Own a copy.
            auto owned = std::make_shared<Message const>(message);
            context->post(
                [state = this->shared_from_this(), owned = std::move(owned)] { state->received(owned); });
        } catch (...) {
            finish(lexec::set_error, std::current_exception());
        }
    }
    void received(std::shared_ptr<Message const> const &message) noexcept {
        std::exception_ptr error;
        auto stopped = false;
        {
            auto const lock = std::lock_guard<std::recursive_mutex>{mutex};
            // A queued message can outlive the receiver and its stop source.
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
            finish(lexec::set_value, message);
    }
    void failedPost(std::exception_ptr error) noexcept {
        {
            auto const lock = std::lock_guard<std::recursive_mutex>{mutex};
            completion.rememberError(error);
            if (not initialized)
                return;
        }
        finish(lexec::set_error, error);
    }
    template <class Tag, class... Values> void finish(Tag tag, Values &&...values) noexcept {
        std::unique_ptr<Receiver> receiver;
        typename rclcpp::Subscription<Message>::SharedPtr detached;
        {
            auto const lock = std::lock_guard<std::recursive_mutex>{mutex};
            receiver = completion.takeReceiver();
            if (not receiver)
                return;
            detached = std::exchange(subscription, {});
        }
        stopCallback.reset();
        if (detached) {
            detached.reset();
            try {
                // A failed stop post may complete outside the executor while
                // its wait set still holds a temporary subscription owner.
                context->node().get_node_base_interface()->trigger_notify_guard_condition();
            } catch (...) {
                lexec::set_error(std::move(*receiver), std::current_exception());
                return;
            }
        }
        tag(std::move(*receiver), std::forward<Values>(values)...);
    }

    std::shared_ptr<ExecutionContext> context;
    std::string topic;
    rclcpp::QoS qos;
    Token token;
    Completion<Receiver> completion;
    std::optional<StopCallback> stopCallback;
    typename rclcpp::Subscription<Message>::SharedPtr subscription;
    std::recursive_mutex mutex;
    bool initialized = false;
    bool messageClaimed = false;
};

} // namespace detail

template <class Message> struct TopicSender {
    using sender_concept = lexec::sender_t;
    using completion_signatures =
        lexec::completion_signatures<lexec::set_value_t(std::shared_ptr<Message const>),
                                     lexec::set_error_t(std::exception_ptr), lexec::set_stopped_t()>;

    template <class Receiver> auto connect(Receiver receiver) const & {
        using State = detail::TopicState<Message, Receiver>;
        return detail::SharedOperation<State>{
            std::make_shared<State>(context, topic, qos, std::move(receiver))};
    }
    template <class Receiver> auto connect(Receiver receiver) && {
        using State = detail::TopicState<Message, Receiver>;
        return detail::SharedOperation<State>{
            std::make_shared<State>(context, std::move(topic), qos, std::move(receiver))};
    }

    std::shared_ptr<detail::ExecutionContext> context;
    std::string topic;
    rclcpp::QoS qos;
};

// Each start subscribes independently. QoS may deliver retained older messages.
template <class Message>
auto wait_message(TimerScheduler const &scheduler, std::string topic, rclcpp::QoS qos = rclcpp::QoS{1}) {
    return TopicSender<Message>{scheduler.executionContext(), std::move(topic), std::move(qos)};
}

} // namespace lrclexec
