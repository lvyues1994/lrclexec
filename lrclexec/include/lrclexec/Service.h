#pragma once

#include <cstdint>
#include <lrclexec/TimerScheduler.h>
#include <rclcpp/client.hpp>
#include <stdexcept>

namespace lrclexec {
namespace detail {

template <class Service, class Receiver>
struct ServiceState final : std::enable_shared_from_this<ServiceState<Service, Receiver>> {
    using Client = rclcpp::Client<Service>;
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
        std::weak_ptr<ServiceState> weak;
    };
    using StopCallback = lexec::stop_callback_for_t<Token, Stop>;

    ServiceState(std::shared_ptr<ExecutionContext> context_, std::shared_ptr<Client> client_,
                 typename Service::Request request_, Receiver receiver)
        : context{std::move(context_)}, client{std::move(client_)}, request{std::move(request_)},
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
        {
            auto const lock = std::lock_guard<std::recursive_mutex>{mutex};
            try {
                stopCallback.emplace(token, Stop{this->weak_from_this()});
            } catch (...) {
                completion.rememberError(std::current_exception());
            }
            initialized = true;
            error = completion.pendingError();
        }
        if (error)
            finish(lexec::set_error, error);
        else
            poll();
    }
    void poll() noexcept {
        std::exception_ptr error;
        auto stopped = false;
        {
            auto const lock = std::lock_guard<std::recursive_mutex>{mutex};
            if (completion.isComplete() or requestId)
                return;
            error = completion.pendingError();
            stopped = token.stop_requested();
            if (not error and not stopped) {
                try {
                    if (client->service_is_ready())
                        send();
                    else if (not discoveryTimer)
                        armDiscovery();
                } catch (...) {
                    error = std::current_exception();
                }
            }
        }
        if (error)
            finish(lexec::set_error, error);
        else if (stopped)
            finish(lexec::set_stopped);
    }
    void armDiscovery() {
        auto const weak = this->weak_from_this();
        auto &node = context->node();
        discoveryTimer = rclcpp::create_wall_timer(
            std::chrono::milliseconds{20},
            [weak] {
                if (auto state = weak.lock())
                    state->poll();
            },
            context->callbackGroup(), node.get_node_base_interface().get(),
            node.get_node_timers_interface().get());
    }
    void send() {
        if (discoveryTimer) {
            discoveryTimer->cancel();
            discoveryTimer.reset();
        }
        auto const weak = this->weak_from_this();
        auto future = client->async_send_request(
            std::make_shared<typename Service::Request>(std::move(request)),
            [weak](typename Client::SharedFuture response) {
                if (auto state = weak.lock()) {
                    try {
                        state->context->post([state, response] { state->responded(response); });
                    } catch (...) {
                        state->finish(lexec::set_error, std::current_exception());
                    }
                }
            });
        requestId = future.request_id;
    }
    void responded(typename Client::SharedFuture const &response) noexcept {
        std::exception_ptr error;
        auto stopped = false;
        {
            auto const lock = std::lock_guard<std::recursive_mutex>{mutex};
            if (completion.isComplete())
                return;
            error = completion.pendingError();
            stopped = token.stop_requested();
        }
        try {
            if (error)
                finish(lexec::set_error, error);
            else if (stopped)
                finish(lexec::set_stopped);
            else
                finish(lexec::set_value, response.get());
        } catch (...) {
            finish(lexec::set_error, std::current_exception());
        }
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
        rclcpp::TimerBase::SharedPtr timer;
        std::optional<std::int64_t> id;
        {
            auto const lock = std::lock_guard<std::recursive_mutex>{mutex};
            receiver = completion.takeReceiver();
            if (not receiver)
                return;
            timer = std::exchange(discoveryTimer, {});
            id = std::exchange(requestId, {});
        }
        stopCallback.reset();
        try {
            if (timer)
                timer->cancel();
            if (id)
                (void)client->remove_pending_request(*id);
        } catch (...) {
            lexec::set_error(std::move(*receiver), std::current_exception());
            return;
        }
        tag(std::move(*receiver), std::forward<Values>(values)...);
    }

    std::shared_ptr<ExecutionContext> context;
    std::shared_ptr<Client> client;
    typename Service::Request request;
    Token token;
    Completion<Receiver> completion;
    std::optional<StopCallback> stopCallback;
    rclcpp::TimerBase::SharedPtr discoveryTimer;
    std::optional<std::int64_t> requestId;
    std::recursive_mutex mutex;
    bool initialized = false;
};

} // namespace detail

template <class Service> struct ServiceSender {
    using sender_concept = lexec::sender_t;
    using completion_signatures =
        lexec::completion_signatures<lexec::set_value_t(typename Service::Response::SharedPtr),
                                     lexec::set_error_t(std::exception_ptr), lexec::set_stopped_t()>;

    template <class Receiver> auto connect(Receiver receiver) const & {
        using State = detail::ServiceState<Service, Receiver>;
        return detail::SharedOperation<State>{
            std::make_shared<State>(context, client, request, std::move(receiver))};
    }
    template <class Receiver> auto connect(Receiver receiver) && {
        using State = detail::ServiceState<Service, Receiver>;
        return detail::SharedOperation<State>{
            std::make_shared<State>(context, client, std::move(request), std::move(receiver))};
    }

    std::shared_ptr<detail::ExecutionContext> context;
    std::shared_ptr<rclcpp::Client<Service>> client;
    typename Service::Request request;
};

// Discovery uses wall time so this also works before the first /clock message.
// Stop detaches the local request; ROS services cannot cancel remote execution.
template <class Service>
auto call_service(TimerScheduler const &scheduler, std::shared_ptr<rclcpp::Client<Service>> client,
                  typename Service::Request request) {
    if (not client)
        throw std::invalid_argument{"lrclexec: call_service needs a client"};
    return ServiceSender<Service>{scheduler.executionContext(), std::move(client), std::move(request)};
}

} // namespace lrclexec
