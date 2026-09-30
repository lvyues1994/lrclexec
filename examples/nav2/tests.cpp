#include "MockRobot.h"
#include <iostream>
#include <lexec/coro/co2.hpp>
#include <lrclexec/SpinWithScope.h>
#include <rclcpp/rclcpp.hpp>
#include <rclcpp_action/create_client.hpp>

using namespace std::chrono_literals;

namespace {

void require(bool condition, char const *message) {
    if (not condition)
        throw std::runtime_error{message};
}
enum struct StopPoint { none, initialPlan, following, replanning };
struct Outcome {
    bool reached = false;
    bool stopped = false;
    std::exception_ptr error;
    navigation::MockStats stats;
    std::size_t peakPaths = 0;
};

Outcome run(navigation::MockConfig config = {}, StopPoint point = StopPoint::none,
            std::chrono::nanoseconds interval = 10ms) {
    static auto index = 0;
    auto node = std::make_shared<rclcpp::Node>("navigation_test", "/nav2_case_" + std::to_string(++index));
    auto scheduler = lrclexec::TimerScheduler{node};
    auto robot = navigation::MockRobot{scheduler, config};
    auto resources = navigation::Resources{
        scheduler, rclcpp_action::create_client<navigation::Plan>(node, "compute_path_to_pose"),
        rclcpp_action::create_client<navigation::Follow>(node, "follow_path")};
    require(resources.planner->wait_for_action_server(3s), "planner discovery failed");
    require(resources.controller->wait_for_action_server(3s), "controller discovery failed");
    resources.replanInterval = interval;
    auto outcome = Outcome{};
    auto paths = std::vector<std::weak_ptr<navigation::Plan::Result>>{};
    resources.pathReady = [&](auto const &path) {
        paths.erase(std::remove_if(paths.begin(), paths.end(), [](auto const &old) { return old.expired(); }),
                    paths.end());
        paths.push_back(path);
        outcome.peakPaths = std::max(outcome.peakPaths, paths.size());
    };
    auto target = geometry_msgs::msg::PoseStamped{};
    target.header.frame_id = "map";
    target.pose.orientation.w = 1.0;
    auto stop = lexec::inplace_stop_source{};
    auto scope = lexec::counting_scope{};
    auto work = lexec::coro::as_sender(navigation::navigate(resources, target)) | lexec::then([&]() noexcept {
                    outcome.reached = true;
                    stop.request_stop();
                }) |
                lexec::upon_error([&](std::exception_ptr error) noexcept {
                    outcome.error = error;
                    stop.request_stop();
                }) |
                lexec::upon_stopped([&]() noexcept {
                    outcome.stopped = true;
                    stop.request_stop();
                });
    lexec::spawn(std::move(work), scope.get_token());
    auto stopTimer = rclcpp::create_wall_timer(
        1ms,
        [&] {
            auto const stats = robot.stats();
            if ((point == StopPoint::initialPlan and stats.activePlans == 1 and stats.plans == 1) or
                (point == StopPoint::following and stats.activeFollows == 1) or
                (point == StopPoint::replanning and stats.activePlans == 1 and stats.plans == 2))
                stop.request_stop();
        },
        scheduler.callbackGroup(), node->get_node_base_interface().get(),
        node->get_node_timers_interface().get());
    auto executor = rclcpp::executors::SingleThreadedExecutor{};
    executor.add_node(node);
    lrclexec::spin_with_scope(executor, scope, stop.get_token());
    stopTimer->cancel();
    outcome.stats = robot.stats();
    require(outcome.stats.activePlans == 0 and outcome.stats.activeFollows == 0, "join retained remote work");
    require(outcome.stats.peakFollows <= 1, "new controller started before old controller drained");
    require(std::all_of(paths.begin(), paths.end(), [](auto const &path) { return path.expired(); }),
            "joined navigation retained a path result");
    return outcome;
}

navigation::Failure failure(Outcome const &outcome) {
    require(bool(outcome.error), "missing expected navigation error");
    try {
        std::rethrow_exception(outcome.error);
    } catch (navigation::NavigationError const &error) {
        return error.failure;
    }
    throw std::runtime_error{"wrong error type"};
}

void successAndBoundedLoop() {
    auto config = navigation::MockConfig{};
    config.finishAfterPlans = 300;
    auto const result = run(config, StopPoint::none, 2ms);
    require(result.reached and not result.error and not result.stopped, "navigation did not succeed");
    // The final result travels asynchronously: a new planner can be accepted
    // before the controller's success reaches when_any, then lose that race.
    require(result.stats.follows >= 300 and result.stats.plans >= result.stats.follows,
            "long-running replan loop did not execute");
    require(result.stats.canceledFollows == result.stats.follows - 1, "losing controllers were not canceled");
    require(result.peakPaths <= 3, "path retention grew across replan iterations");
    std::cout << result.stats.plans << " plans, peak live client paths: " << result.peakPaths << '\n';
}

void resultErrors() {
    auto config = navigation::MockConfig{};
    config.planError = navigation::Plan::Result::NO_VALID_PATH;
    auto result = run(config);
    auto error = failure(result);
    require(error.phase == navigation::Phase::planning and not error.action and
                error.code == navigation::Plan::Result::NO_VALID_PATH and result.stats.follows == 0,
            "SUCCEEDED planner business failure was lost");
    config.planErrorFrom = 2;
    result = run(config);
    require(failure(result).phase == navigation::Phase::planning and result.stats.canceledFollows == 1,
            "replanning failure did not drain the running controller");
    config = {};
    config.followError = navigation::Follow::Result::FAILED_TO_MAKE_PROGRESS;
    result = run(config);
    error = failure(result);
    require(error.phase == navigation::Phase::following and not error.action and
                error.code == navigation::Follow::Result::FAILED_TO_MAKE_PROGRESS,
            "SUCCEEDED controller business failure was lost");
    config.abortFollow = true;
    error = failure(run(config));
    require(error.action == lrclexec::ActionErrorKind::aborted and
                error.code == navigation::Follow::Result::FAILED_TO_MAKE_PROGRESS and
                error.message == "mock controller failure",
            "ABORTED payload was lost");
    config = {};
    config.rejectPlan = true;
    error = failure(run(config));
    require(error.action == lrclexec::ActionErrorKind::rejected and not error.code, "rejection was lost");
    config = {};
    config.emptyPath = true;
    result = run(config);
    require(failure(result).phase == navigation::Phase::planning and result.stats.follows == 0,
            "empty path reached controller");
}

void cancellation() {
    auto config = navigation::MockConfig{};
    config.planDelay = 30ms;
    auto result = run(config, StopPoint::initialPlan);
    require(result.stopped and not result.error and result.stats.canceledPlans == 1,
            "initial plan cancellation failed");
    config = {};
    result = run(config, StopPoint::following, 100ms);
    require(result.stopped and not result.error and result.stats.canceledFollows == 1 and
                result.stats.plans == 1,
            "follow/delay cancellation failed");
    config.planDelay = 30ms;
    result = run(config, StopPoint::replanning);
    require(result.stopped and not result.error and result.stats.canceledPlans == 1 and
                result.stats.canceledFollows == 1,
            "concurrent follow/replan cancellation failed");
}

void successDrainsPlanner() {
    auto config = navigation::MockConfig{};
    config.finishAfterPlans = 1;
    config.planDelay = 30ms;
    config.followDelay = 20ms;
    auto const result = run(config, StopPoint::none, 5ms);
    require(result.reached and not result.error and result.stats.canceledPlans == 1,
            "successful controller did not drain the running planner");
}

} // namespace

int main(int argc, char **argv) {
    rclcpp::init(argc, argv, rclcpp::InitOptions{}, rclcpp::SignalHandlerOptions::None);
    auto status = 0;
    try {
        successAndBoundedLoop();
        resultErrors();
        cancellation();
        successDrainsPlanner();
        std::cout << "Nav2 message regression tests passed\n";
    } catch (std::exception const &error) {
        std::cerr << error.what() << '\n';
        status = 1;
    }
    rclcpp::shutdown();
    return status;
}
