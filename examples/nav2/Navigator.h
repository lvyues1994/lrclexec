#pragma once

#include <chrono>
#include <co2/task.hpp>
#include <functional>
#include <lrclexec/ExecuteAction.h>
#include <nav2_msgs/action/compute_path_to_pose.hpp>
#include <nav2_msgs/action/follow_path.hpp>
#include <optional>
#include <stdexcept>
#include <string>

namespace navigation {

using Plan = nav2_msgs::action::ComputePathToPose;
using Follow = nav2_msgs::action::FollowPath;

enum struct Phase { planning, following };
struct Failure {
    Phase phase;
    std::optional<lrclexec::ActionErrorKind> action;
    std::optional<std::uint16_t> code;
    std::string message;
};
struct NavigationError : std::runtime_error {
    explicit NavigationError(Failure failure_)
        : std::runtime_error{failure_.message}, failure{std::move(failure_)} {}
    Failure const failure;
};

struct Resources {
    lrclexec::TimerScheduler scheduler;
    rclcpp_action::Client<Plan>::SharedPtr planner;
    rclcpp_action::Client<Follow>::SharedPtr controller;
    std::chrono::nanoseconds replanInterval = std::chrono::seconds{1};
    std::string plannerId;
    std::string controllerId;
    std::string goalCheckerId;
    std::string progressCheckerId;
    std::function<void(Plan::Result::SharedPtr const &)> pathReady;
    std::function<void(std::shared_ptr<Follow::Feedback const>)> progress;
};

// Each iteration awaits one race. when_any drains the losing Action before
// the loop replaces its path and starts the next controller.
co2::Task<> navigate(Resources resources, geometry_msgs::msg::PoseStamped target);

} // namespace navigation
