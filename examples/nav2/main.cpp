#include "MockRobot.h"
#include <iostream>
#include <lexec/coro/co2.hpp>
#include <lrclexec/SignalStop.h>
#include <lrclexec/SpinWithScope.h>
#include <rclcpp/rclcpp.hpp>
#include <rclcpp_action/create_client.hpp>

int main(int argc, char **argv) {
    auto stop = lexec::inplace_stop_source{};
    auto signals = lrclexec::SignalStop{stop};
    auto options = rclcpp::InitOptions{};
    options.shutdown_on_signal = false;
    rclcpp::init(argc, argv, options, rclcpp::SignalHandlerOptions::None);
    auto node = std::make_shared<rclcpp::Node>("lrclexec_nav2_example");
    auto scheduler = lrclexec::TimerScheduler{node};
    auto robot = navigation::MockRobot{scheduler};
    auto resources = navigation::Resources{
        scheduler, rclcpp_action::create_client<navigation::Plan>(node, "compute_path_to_pose"),
        rclcpp_action::create_client<navigation::Follow>(node, "follow_path")};
    if (not resources.planner->wait_for_action_server(std::chrono::seconds{3}) or
        not resources.controller->wait_for_action_server(std::chrono::seconds{3})) {
        std::cerr << "local Nav2 action discovery failed\n";
        rclcpp::shutdown();
        return 1;
    }
    resources.replanInterval = std::chrono::milliseconds{20};
    resources.pathReady = [](auto const &path) {
        std::cout << "received path with " << path->path.poses.size() << " poses\n";
    };
    auto target = geometry_msgs::msg::PoseStamped{};
    target.header.frame_id = "map";
    target.pose.position.x = 1.0;
    target.pose.orientation.w = 1.0;
    auto failed = false;
    auto reached = false;
    auto scope = lexec::counting_scope{};
    auto work = lexec::coro::as_sender(navigation::navigate(resources, target)) | lexec::then([&]() noexcept {
                    reached = true;
                    std::cout << "reached goal\n";
                    stop.request_stop();
                }) |
                lexec::upon_error([&](std::exception_ptr error) noexcept {
                    failed = true;
                    try {
                        std::rethrow_exception(error);
                    } catch (std::exception const &exception) {
                        std::cerr << exception.what() << '\n';
                    }
                    stop.request_stop();
                }) |
                lexec::upon_stopped([&]() noexcept { stop.request_stop(); });
    lexec::spawn(std::move(work), scope.get_token());
    auto executor = rclcpp::executors::SingleThreadedExecutor{};
    executor.add_node(node);
    lrclexec::spin_with_scope(executor, scope, stop.get_token());
    auto const stats = robot.stats();
    rclcpp::shutdown();
    return failed or signals.error() or stats.activePlans or stats.activeFollows or
                   (reached and stats.follows < 3)
               ? 1
               : 0;
}
