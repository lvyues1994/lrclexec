#pragma once

#include "Navigator.h"
#include <algorithm>
#include <rclcpp_action/create_server.hpp>
#include <vector>

namespace navigation {

struct MockConfig {
    int finishAfterPlans = 3;
    std::chrono::nanoseconds planDelay = std::chrono::milliseconds{2};
    std::chrono::nanoseconds followDelay = std::chrono::milliseconds{2};
    std::uint16_t planError = Plan::Result::NONE;
    int planErrorFrom = 1;
    std::uint16_t followError = Follow::Result::NONE;
    bool rejectPlan = false;
    bool abortFollow = false;
    bool emptyPath = false;
};
struct MockStats {
    int plans = 0;
    int follows = 0;
    int activePlans = 0;
    int activeFollows = 0;
    int peakFollows = 0;
    int canceledPlans = 0;
    int canceledFollows = 0;
};

// The mock owns all timers and servers. Callbacks borrow them weakly; a
// terminal response cancels and removes its timer instead of retaining history.
class MockRobot {
    template <class Action> struct Slot {
        std::shared_ptr<rclcpp_action::ServerGoalHandle<Action>> handle;
        typename Action::Result::SharedPtr result;
        rclcpp::TimerBase::SharedPtr timer;
        std::chrono::steady_clock::time_point finishAt;
        std::optional<std::chrono::steady_clock::time_point> cancelAt;
        bool abort = false;
    };
    struct State : std::enable_shared_from_this<State> {
        State(lrclexec::TimerScheduler scheduler_, MockConfig config_)
            : scheduler{std::move(scheduler_)}, config{config_} {}

        template <class Action>
        void arm(std::shared_ptr<Slot<Action>> slot, std::vector<std::shared_ptr<Slot<Action>>> &slots,
                 int &active, int &canceled) {
            ++active;
            slots.push_back(slot);
            auto const weak = this->weak_from_this();
            auto const weakSlot = std::weak_ptr<Slot<Action>>{slot};
            slot->timer = rclcpp::create_wall_timer(
                std::chrono::milliseconds{1},
                [weak, weakSlot, slotsPtr = &slots, activePtr = &active, canceledPtr = &canceled] {
                    if (auto state = weak.lock())
                        if (auto current = weakSlot.lock())
                            state->tick(current, *slotsPtr, *activePtr, *canceledPtr);
                },
                scheduler.callbackGroup(), scheduler.node().get_node_base_interface().get(),
                scheduler.node().get_node_timers_interface().get());
        }
        template <class Action>
        void tick(std::shared_ptr<Slot<Action>> const &slot,
                  std::vector<std::shared_ptr<Slot<Action>>> &slots, int &active, int &canceled) {
            auto const now = std::chrono::steady_clock::now();
            if (slot->handle->is_canceling()) {
                if (not slot->cancelAt)
                    slot->cancelAt = now + std::chrono::milliseconds{3};
                if (now < *slot->cancelAt)
                    return;
                slot->handle->canceled(slot->result);
                ++canceled;
            } else {
                if (now < slot->finishAt)
                    return;
                if (slot->abort)
                    slot->handle->abort(slot->result);
                else
                    slot->handle->succeed(slot->result);
            }
            slot->timer->cancel();
            --active;
            slots.erase(std::remove(slots.begin(), slots.end(), slot), slots.end());
        }
        void acceptPlan(std::shared_ptr<rclcpp_action::ServerGoalHandle<Plan>> handle) {
            ++stats.plans;
            auto slot = std::make_shared<Slot<Plan>>();
            slot->handle = std::move(handle);
            slot->result = std::make_shared<Plan::Result>();
            slot->result->error_code =
                stats.plans >= config.planErrorFrom ? config.planError : Plan::Result::NONE;
            slot->result->error_msg = slot->result->error_code ? "mock planner failure" : "";
            slot->result->path.header = slot->handle->get_goal()->goal.header;
            if (not config.emptyPath)
                slot->result->path.poses.push_back(slot->handle->get_goal()->goal);
            slot->finishAt = std::chrono::steady_clock::now() + config.planDelay;
            arm(slot, plans, stats.activePlans, stats.canceledPlans);
        }
        void acceptFollow(std::shared_ptr<rclcpp_action::ServerGoalHandle<Follow>> handle) {
            ++stats.follows;
            auto slot = std::make_shared<Slot<Follow>>();
            slot->handle = std::move(handle);
            slot->result = std::make_shared<Follow::Result>();
            slot->result->error_code = config.followError;
            slot->result->error_msg = config.followError ? "mock controller failure" : "";
            slot->abort = config.abortFollow;
            auto const delay = stats.plans >= config.finishAfterPlans or config.followError
                                   ? config.followDelay
                                   : std::chrono::seconds{10};
            slot->finishAt = std::chrono::steady_clock::now() + delay;
            arm(slot, follows, stats.activeFollows, stats.canceledFollows);
            stats.peakFollows = std::max(stats.peakFollows, stats.activeFollows);
        }

        lrclexec::TimerScheduler scheduler;
        MockConfig config;
        MockStats stats;
        rclcpp_action::Server<Plan>::SharedPtr planner;
        rclcpp_action::Server<Follow>::SharedPtr controller;
        std::vector<std::shared_ptr<Slot<Plan>>> plans;
        std::vector<std::shared_ptr<Slot<Follow>>> follows;
    };

  public:
    MockRobot(lrclexec::TimerScheduler scheduler, MockConfig config = {})
        : state{std::make_shared<State>(std::move(scheduler), config)} {
        auto const weak = std::weak_ptr<State>{state};
        auto const cancel = [](auto) { return rclcpp_action::CancelResponse::ACCEPT; };
        auto const options = rcl_action_server_get_default_options();
        auto &node = state->scheduler.node();
        auto const group = state->scheduler.callbackGroup();
        state->planner = rclcpp_action::create_server<Plan>(
            node.get_node_base_interface(), node.get_node_clock_interface(),
            node.get_node_logging_interface(), node.get_node_waitables_interface(), "compute_path_to_pose",
            [weak](auto const &, auto) {
                auto current = weak.lock();
                return current and not current->config.rejectPlan
                           ? rclcpp_action::GoalResponse::ACCEPT_AND_EXECUTE
                           : rclcpp_action::GoalResponse::REJECT;
            },
            cancel,
            [weak](auto handle) {
                if (auto current = weak.lock())
                    current->acceptPlan(std::move(handle));
            },
            options, group);
        state->controller = rclcpp_action::create_server<Follow>(
            node.get_node_base_interface(), node.get_node_clock_interface(),
            node.get_node_logging_interface(), node.get_node_waitables_interface(), "follow_path",
            [](auto const &, auto) { return rclcpp_action::GoalResponse::ACCEPT_AND_EXECUTE; }, cancel,
            [weak](auto handle) {
                if (auto current = weak.lock())
                    current->acceptFollow(std::move(handle));
            },
            options, group);
    }
    MockStats stats() const { return state->stats; }

  private:
    std::shared_ptr<State> state;
};

} // namespace navigation
