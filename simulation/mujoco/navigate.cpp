#include "Navigator.h"
#include <atomic>
#include <chrono>
#include <cmath>
#include <iostream>
#include <lexec/coro/co2.hpp>
#include <lifecycle_msgs/msg/state.hpp>
#include <lifecycle_msgs/srv/get_state.hpp>
#include <lrclexec/Service.h>
#include <lrclexec/SignalStop.h>
#include <mutex>
#include <nav2_msgs/srv/set_initial_pose.hpp>
#include <rclcpp/rclcpp.hpp>
#include <rclcpp_action/create_client.hpp>
#include <std_srvs/srv/empty.hpp>
#include <tf2_ros/buffer.h>
#include <tf2_ros/transform_listener.h>
#include <thread>

using namespace std::chrono_literals;
namespace {
struct StartupStopped {};
// Keep every ROS owner alive until spin() has returned, including failure paths.
class Spinning {
  public:
    explicit Spinning(rclcpp::Executor &executor_)
        : executor{executor_}, thread{[this] {
              try {
                  executor.spin();
              } catch (std::exception const &error) {
                  std::cerr << "executor failed: " << error.what() << '\n';
                  std::_Exit(125); // Pending senders still borrow their receivers.
              }
          }} {
        auto const deadline = std::chrono::steady_clock::now() + 5s;
        while (not executor.is_spinning()) {
            if (std::chrono::steady_clock::now() >= deadline) {
                std::cerr << "executor failed to start\n";
                std::_Exit(125);
            }
            std::this_thread::sleep_for(1ms);
        }
    }
    ~Spinning() { stop(); }
    Spinning(Spinning const &) = delete;
    Spinning &operator=(Spinning const &) = delete;
    void stop() {
        if (thread.joinable()) {
            executor.cancel();
            thread.join();
        }
    }

  private:
    rclcpp::Executor &executor;
    std::thread thread;
};
template <class Sender>
bool timed(lrclexec::TimerScheduler const &scheduler, Sender sender, lexec::inplace_stop_token stop) {
    auto timeout = lrclexec::schedule_after(scheduler, 500ms) | lexec::then([]() noexcept { return false; });
    auto result = lexec::sync_wait(lexec::write_env(lexec::when_any(std::move(sender), std::move(timeout)),
                                                    lexec::prop{lexec::get_stop_token, stop}));
    if (not result)
        throw StartupStopped{};
    return std::get<0>(*result);
}
void waitActive(lrclexec::TimerScheduler const &scheduler,
                rclcpp::Client<lifecycle_msgs::srv::GetState>::SharedPtr const &client,
                lexec::inplace_stop_token stop) {
    auto const deadline = std::chrono::steady_clock::now() + 25s;
    while (std::chrono::steady_clock::now() < deadline) {
        auto query =
            lrclexec::call_service(scheduler, client, lifecycle_msgs::srv::GetState::Request{}) |
            lexec::then([](auto response) noexcept {
                return response->current_state.id == lifecycle_msgs::msg::State::PRIMARY_STATE_ACTIVE;
            });
        if (timed(scheduler, std::move(query), stop))
            return;
        auto delay = lexec::sync_wait(lexec::write_env(lrclexec::schedule_after(scheduler, 100ms),
                                                       lexec::prop{lexec::get_stop_token, stop}));
        if (not delay)
            throw StartupStopped{};
    }
    throw std::runtime_error{"Nav2 lifecycle did not become ACTIVE"};
}
struct Localization {
    void observe(geometry_msgs::msg::PoseWithCovarianceStamped const &message) {
        auto lock = std::lock_guard{mutex};
        latest = message;
        ++samples;
    }
    bool ready(rclcpp::Time const now) {
        auto lock = std::lock_guard{mutex};
        auto const age = (now - rclcpp::Time{latest.header.stamp}).seconds();
        auto const &c = latest.pose.covariance;
        return samples >= 10 and latest.header.frame_id == "map" and age >= 0 and age < 1.0 and
               std::isfinite(c[0]) and std::isfinite(c[7]) and std::isfinite(c[35]) and c[0] >= 0 and
               c[7] >= 0 and c[35] >= 0 and c[0] + c[7] < .02 and c[35] < .02;
    }
    std::mutex mutex;
    geometry_msgs::msg::PoseWithCovarianceStamped latest{};
    unsigned samples = 0;
};
} // namespace

int main(int argc, char **argv) {
    auto stop = lexec::inplace_stop_source{};
    auto signals = lrclexec::SignalStop{stop};
    auto options = rclcpp::InitOptions{};
    options.shutdown_on_signal = false;
    rclcpp::init(argc, argv, options, rclcpp::SignalHandlerOptions::None);
    auto status = 0;
    try {
        auto nodeOptions =
            rclcpp::NodeOptions{}.parameter_overrides({rclcpp::Parameter{"use_sim_time", true}});
        auto node = std::make_shared<rclcpp::Node>("lexec_mujoco_navigator", nodeOptions);
        auto const mode = node->declare_parameter<std::string>("executor", "single");
        auto const localizationMode = node->declare_parameter<std::string>("localization", "amcl");
        if (localizationMode != "amcl" and localizationMode != "truth")
            throw std::invalid_argument{"localization must be amcl or truth"};
        std::unique_ptr<rclcpp::Executor> executor;
        if (mode == "single")
            executor = std::make_unique<rclcpp::executors::SingleThreadedExecutor>();
        else if (mode == "multi")
            executor =
                std::make_unique<rclcpp::executors::MultiThreadedExecutor>(rclcpp::ExecutorOptions{}, 4);
        else
            throw std::invalid_argument{"executor must be single or multi"};
        executor->add_node(node);
        auto startupScheduler = lrclexec::TimerScheduler{node};
        auto scheduler = lrclexec::TimerScheduler{node, lrclexec::TimerClock::node};
        using GetState = lifecycle_msgs::srv::GetState;
        auto amclState = node->create_client<GetState>("amcl/get_state");
        auto plannerState = node->create_client<GetState>("planner_server/get_state");
        auto controllerState = node->create_client<GetState>("controller_server/get_state");
        auto initialPose = node->create_client<nav2_msgs::srv::SetInitialPose>("set_initial_pose");
        auto update = node->create_client<std_srvs::srv::Empty>("request_nomotion_update");
        auto localization = Localization{};
        auto poses = node->create_subscription<geometry_msgs::msg::PoseWithCovarianceStamped>(
            "amcl_pose", rclcpp::QoS{10}.transient_local(),
            [&](geometry_msgs::msg::PoseWithCovarianceStamped const &message) {
                localization.observe(message);
            });
        auto transforms = tf2_ros::Buffer{node->get_clock()};
        auto listener = tf2_ros::TransformListener{transforms, node, false};
        auto resources = navigation::Resources{
            scheduler, rclcpp_action::create_client<navigation::Plan>(node, "compute_path_to_pose"),
            rclcpp_action::create_client<navigation::Follow>(node, "follow_path")};
        auto target = geometry_msgs::msg::PoseStamped{};
        target.header.frame_id = "map";
        target.pose.position.x = node->declare_parameter<double>("target_x", 4.0);
        target.pose.position.y = node->declare_parameter<double>("target_y", 0.0);
        target.pose.orientation.w = 1;
        auto const initialX = node->declare_parameter<double>("initial_x", .15);
        auto const initialY = node->declare_parameter<double>("initial_y", -.10);
        auto const initialYaw = node->declare_parameter<double>("initial_yaw", .08);
        auto spinner = Spinning{*executor};
        std::cout << "WAITING_FOR_NAV2 executor=" << mode << " localization=" << localizationMode << '\n'
                  << std::flush;
        if (localizationMode == "amcl") {
            waitActive(startupScheduler, amclState, stop.get_token());
            auto request = nav2_msgs::srv::SetInitialPose::Request{};
            request.pose.header.frame_id = "map";
            request.pose.header.stamp = node->now();
            request.pose.pose.pose.position.x = initialX;
            request.pose.pose.pose.position.y = initialY;
            request.pose.pose.pose.orientation.z = std::sin(initialYaw / 2);
            request.pose.pose.pose.orientation.w = std::cos(initialYaw / 2);
            request.pose.pose.covariance[0] = .09;
            request.pose.pose.covariance[7] = .09;
            request.pose.pose.covariance[35] = .04;
            auto initialized = timed(startupScheduler,
                                     lrclexec::call_service(startupScheduler, initialPose, request) |
                                         lexec::then([](auto) noexcept { return true; }),
                                     stop.get_token());
            if (not initialized)
                throw std::runtime_error{"AMCL initial pose service timed out"};
            auto const deadline = std::chrono::steady_clock::now() + 20s;
            while (not localization.ready(node->now()) or
                   not transforms.canTransform("map", "base_link", tf2::TimePointZero)) {
                if (std::chrono::steady_clock::now() >= deadline)
                    throw std::runtime_error{"AMCL localization did not become ready"};
                (void)timed(
                    startupScheduler,
                    lrclexec::call_service(startupScheduler, update, std_srvs::srv::Empty::Request{}) |
                        lexec::then([](auto) noexcept { return true; }),
                    stop.get_token());
                if (not lexec::sync_wait(
                        lexec::write_env(lrclexec::schedule_after(startupScheduler, 200ms),
                                         lexec::prop{lexec::get_stop_token, stop.get_token()})))
                    throw StartupStopped{};
            }
            std::cout << "LOCALIZATION_READY time=" << node->now().seconds() << '\n' << std::flush;
        }
        waitActive(startupScheduler, plannerState, stop.get_token());
        waitActive(startupScheduler, controllerState, stop.get_token());
        if (not resources.planner->wait_for_action_server(3s) or
            not resources.controller->wait_for_action_server(3s))
            throw std::runtime_error{"Nav2 action discovery failed"};
        resources.plannerId = "GridBased";
        resources.controllerId = "FollowPath";
        resources.goalCheckerId = "goal_checker";
        resources.progressCheckerId = "progress_checker";
        resources.replanInterval = 1s;
        auto planCount = 0;
        auto progressCount = 0;
        resources.pathReady = [&](auto const &) { ++planCount; };
        resources.progress = [&](auto const &) { ++progressCount; };
        target.header.stamp = node->now();
        auto scope = lexec::counting_scope{};
        auto work = lexec::coro::as_sender(navigation::navigate(resources, target)) |
                    lexec::then([&]() noexcept {
                        std::cout << "SUCCEEDED\n";
                        stop.request_stop();
                    }) |
                    lexec::upon_stopped([&]() noexcept {
                        std::cout << "STOPPED\n";
                        stop.request_stop();
                    }) |
                    lexec::upon_error([&](std::exception_ptr error) noexcept {
                        status = 1;
                        try {
                            std::rethrow_exception(error);
                        } catch (std::exception const &exception) {
                            std::cerr << exception.what() << '\n';
                        }
                        stop.request_stop();
                    });
        std::cout << "GOAL_STARTED time=" << target.header.stamp.sec << '\n' << std::flush;
        lexec::spawn(std::move(work), scope.get_token());
        while (not stop.stop_requested())
            std::this_thread::sleep_for(10ms);
        scope.close();
        scope.request_stop();
        (void)lexec::sync_wait(scope.join());
        spinner.stop();
        std::cout << "DRAINED plans=" << planCount << " feedback=" << progressCount << '\n' << std::flush;
        if (signals.error())
            status = 1;
    } catch (StartupStopped const &) {
        std::cout << "STOPPED\nDRAINED plans=0\n" << std::flush;
    } catch (std::exception const &error) {
        std::cerr << error.what() << '\n';
        status = 1;
    }
    rclcpp::shutdown();
    return status;
}
