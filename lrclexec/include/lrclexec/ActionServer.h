#pragma once

#include <atomic>
#include <lrclexec/TimerScheduler.h>
#include <mutex>
#include <rclcpp_action/create_server.hpp>
#include <string>
#include <utility>

namespace lrclexec::detail {

using ScopeLease = std::shared_ptr<lexec::counting_scope::association>;

// spawn destroys its whole operation before releasing this association.
template <class State> struct DrainAssociation {
    DrainAssociation() = default;
    DrainAssociation(ScopeLease lease_, std::shared_ptr<State> state_)
        : lease{std::move(lease_)}, state{std::move(state_)} {}
    DrainAssociation(DrainAssociation &&) noexcept = default;
    DrainAssociation &operator=(DrainAssociation &&other) noexcept {
        if (this != &other) {
            release();
            lease = std::move(other.lease);
            state = std::move(other.state);
        }
        return *this;
    }
    ~DrainAssociation() { release(); }
    explicit operator bool() const noexcept { return lease and static_cast<bool>(*lease); }
    DrainAssociation try_associate() const {
        if (not *this)
            return {};
        return {std::make_shared<lexec::counting_scope::association>(lease->try_associate()), {}};
    }

  private:
    void release() noexcept {
        auto kept = std::move(lease);
        if (kept and state)
            state->released(std::move(kept));
    }
    ScopeLease lease;
    std::shared_ptr<State> state;
};

template <class State> struct AdoptAssociation {
    template <class Sender> Sender &&wrap(Sender &&sender) const noexcept {
        return std::forward<Sender>(sender);
    }
    DrainAssociation<State> try_associate() const noexcept { return {std::move(*lease), state}; }
    ScopeLease *lease;
    std::shared_ptr<State> state;
};

template <class Action, class Factory>
struct ActionServerState final : std::enable_shared_from_this<ActionServerState<Action, Factory>> {
    using Handle = rclcpp_action::ServerGoalHandle<Action>;
    using Result = typename Action::Result;
    struct Mission {
        std::shared_ptr<Handle> handle;
        ScopeLease lease;
        std::weak_ptr<lexec::counting_scope::association> tracking;
        lexec::inplace_stop_source stop;
        std::shared_ptr<Result> fallback = std::make_shared<Result>();
        bool terminal = false;
    };

    ActionServerState(TimerScheduler scheduler_, lexec::counting_scope &scope_, Factory factory_)
        : scheduler{std::move(scheduler_)}, scope{scope_}, factory{std::move(factory_)} {}

    void bind(std::string const &name) {
        auto const weak = this->weak_from_this();
        server = rclcpp_action::create_server<Action>(
            scheduler.node().shared_from_this(), name,
            [weak](auto const &, auto const &) {
                auto state = weak.lock();
                if (not state or state->closed.load() or not state->scope.get_token().try_associate())
                    return rclcpp_action::GoalResponse::REJECT;
                return rclcpp_action::GoalResponse::ACCEPT_AND_EXECUTE;
            },
            [weak](std::shared_ptr<Handle> handle) {
                auto state = weak.lock();
                if (not state)
                    return rclcpp_action::CancelResponse::REJECT;
                auto mission = state->find(handle);
                if (not mission)
                    return rclcpp_action::CancelResponse::REJECT;
                // Jazzy enters CANCELING only after this callback returns ACCEPT.
                try {
                    state->scheduler.executionContext()->post(
                        [weak, mission = std::weak_ptr<Mission>{mission}] {
                            if (auto current = weak.lock()) {
                                if (auto goal = mission.lock())
                                    current->cancel(goal);
                            }
                        });
                } catch (...) {
                    return rclcpp_action::CancelResponse::REJECT;
                }
                return rclcpp_action::CancelResponse::ACCEPT;
            },
            [weak](std::shared_ptr<Handle> handle) {
                if (auto state = weak.lock())
                    state->accept(std::move(handle));
            },
            rcl_action_server_get_default_options(), scheduler.callbackGroup());
    }
    void close() {
        if (closed.exchange(true))
            return;
        auto lease = std::make_shared<lexec::counting_scope::association>(scope.get_token().try_associate());
        if (not *lease) {
            auto const lock = std::lock_guard<std::mutex>{mutex};
            if (pending)
                lease = pending->lease;
            else if (active)
                lease = active->tracking.lock();
            else
                return;
        }
        auto state = this->shared_from_this();
        try {
            scheduler.executionContext()->post([state, lease] { state->stopMissions(); });
        } catch (...) {
            stopMissions();
            throw;
        }
    }
    // Keep the old association through the handoff task, including during scope closure.
    void released(ScopeLease lease) noexcept {
        auto state = this->shared_from_this();
        try {
            scheduler.executionContext()->post([state, lease] {
                {
                    auto const lock = std::lock_guard<std::mutex>{state->mutex};
                    state->handoffPending = false;
                }
                state->startPending();
            });
        } catch (...) {
            closed.store(true);
            {
                auto const lock = std::lock_guard<std::mutex>{mutex};
                handoffPending = false;
            }
            stopMissions();
        }
    }

  private:
    std::shared_ptr<Mission> find(std::shared_ptr<Handle> const &handle) const {
        auto const lock = std::lock_guard<std::mutex>{mutex};
        if (active and active->handle == handle)
            return active;
        if (pending and pending->handle == handle)
            return pending;
        return {};
    }
    void cancel(std::shared_ptr<Mission> const &mission) noexcept {
        auto isPending = false;
        {
            auto const lock = std::lock_guard<std::mutex>{mutex};
            isPending = pending == mission;
        }
        mission->stop.request_stop();
        if (isPending)
            finish(mission, mission->fallback, true);
    }
    void stopMissions() noexcept {
        std::shared_ptr<Mission> running, waiting;
        {
            auto const lock = std::lock_guard<std::mutex>{mutex};
            running = active;
            waiting = pending;
        }
        if (waiting)
            finish(waiting, waiting->fallback, true);
        if (running)
            running->stop.request_stop();
    }
    void accept(std::shared_ptr<Handle> handle) {
        auto mission = std::make_shared<Mission>();
        mission->handle = std::move(handle);
        mission->lease =
            std::make_shared<lexec::counting_scope::association>(scope.get_token().try_associate());
        mission->tracking = mission->lease;
        if (closed.load() or not *mission->lease) {
            finish(mission, mission->fallback, true);
            return;
        }
        std::shared_ptr<Mission> superseded, running;
        {
            auto const lock = std::lock_guard<std::mutex>{mutex};
            superseded = std::exchange(pending, mission);
            running = active;
        }
        if (superseded)
            finish(superseded, superseded->fallback, true);
        if (running)
            running->stop.request_stop();
        else
            startPending();
    }
    void startPending() noexcept {
        std::shared_ptr<Mission> mission;
        {
            auto const lock = std::lock_guard<std::mutex>{mutex};
            if (active or handoffPending or not pending)
                return;
            mission = std::exchange(pending, {});
            active = mission;
        }
        auto state = this->shared_from_this();
        try {
            if (closed.load() or not scope.get_token().try_associate()) {
                finish(mission, mission->fallback, true);
                released(std::move(mission->lease));
                return;
            }
            auto child = scope.get_token().wrap(factory(mission->handle));
            auto work = lexec::write_env(std::move(child),
                                         lexec::prop{lexec::get_stop_token, mission->stop.get_token()}) |
                        lexec::continues_on(scheduler) |
                        lexec::then([state, mission](std::shared_ptr<Result> result) noexcept {
                            state->finish(mission, std::move(result), false);
                        }) |
                        lexec::upon_error([state, mission](auto const &) noexcept {
                            state->finish(mission, mission->fallback, true);
                        }) |
                        lexec::upon_stopped(
                            [state, mission]() noexcept { state->finish(mission, mission->fallback, true); });
            lexec::spawn(std::move(work), AdoptAssociation<ActionServerState>{&mission->lease, state});
        } catch (...) {
            finish(mission, mission->fallback, true);
            released(std::move(mission->lease));
        }
    }
    void finish(std::shared_ptr<Mission> mission, std::shared_ptr<Result> result,
                bool const stopped) noexcept {
        {
            auto const lock = std::lock_guard<std::mutex>{mutex};
            if (mission->terminal)
                return;
            mission->terminal = true;
            if (pending == mission)
                pending.reset();
            if (active == mission) {
                active.reset();
                handoffPending = true;
            }
        }
        if (not result)
            result = mission->fallback;
        try {
            if (mission->handle->is_canceling())
                mission->handle->canceled(result);
            else if (stopped)
                mission->handle->abort(result);
            else
                mission->handle->succeed(result);
        } catch (std::exception const &error) {
            RCLCPP_ERROR(scheduler.node().get_logger(), "Action terminal response failed: %s", error.what());
        }
    }

    TimerScheduler scheduler;
    lexec::counting_scope &scope;
    Factory factory;
    typename rclcpp_action::Server<Action>::SharedPtr server;
    std::shared_ptr<Mission> active;
    std::shared_ptr<Mission> pending;
    mutable std::mutex mutex;
    bool handoffPending = false;
    std::atomic<bool> closed{false};
};

} // namespace lrclexec::detail

namespace lrclexec {

// Keep the server, scope and factory's borrowed objects alive through scope
// join and executor shutdown (including joining any spin threads).
template <class Action, class Factory> struct ActionServer {
    explicit ActionServer(std::shared_ptr<detail::ActionServerState<Action, Factory>> state_)
        : state{std::move(state_)} {}
    void close() { state->close(); }

  private:
    std::shared_ptr<detail::ActionServerState<Action, Factory>> state;
};

template <class Action, class Factory>
auto make_action_server_preempt(TimerScheduler scheduler, lexec::counting_scope &scope,
                                std::string const &name, Factory factory) {
    auto state = std::make_shared<detail::ActionServerState<Action, Factory>>(std::move(scheduler), scope,
                                                                              std::move(factory));
    state->bind(name);
    return ActionServer<Action, Factory>{std::move(state)};
}

} // namespace lrclexec
