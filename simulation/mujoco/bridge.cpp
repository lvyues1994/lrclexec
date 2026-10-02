#include "Physics.h"
#include "Viewer.h"
#include <algorithm>
#include <chrono>
#include <cmath>
#include <deque>
#include <fstream>
#include <geometry_msgs/msg/pose_with_covariance_stamped.hpp>
#include <geometry_msgs/msg/twist.hpp>
#include <iomanip>
#include <lrclexec/SignalStop.h>
#include <nav_msgs/msg/occupancy_grid.hpp>
#include <nav_msgs/msg/odometry.hpp>
#include <rclcpp/rclcpp.hpp>
#include <rosgraph_msgs/msg/clock.hpp>
#include <sensor_msgs/msg/laser_scan.hpp>
#include <tf2_msgs/msg/tf_message.hpp>

using namespace std::chrono_literals;
namespace {

geometry_msgs::msg::TransformStamped transform(char const *parent, char const *child,
                                               builtin_interfaces::msg::Time const &stamp) {
    auto result = geometry_msgs::msg::TransformStamped{};
    result.header.stamp = stamp;
    result.header.frame_id = parent;
    result.child_frame_id = child;
    result.transform.rotation.w = 1;
    return result;
}

class Bridge {
  public:
    Bridge(rclcpp::Node &node_, std::unique_ptr<simulation::Physics> physics_,
           std::unique_ptr<simulation::Viewer> viewer_)
        : node{node_}, physics{std::move(physics_)},
          telemetry{node.declare_parameter<std::string>("telemetry", "physics.jsonl")},
          viewer{std::move(viewer_)} {
        if (not telemetry)
            throw std::runtime_error{"cannot open physics telemetry"};
        telemetry << std::setprecision(9);
        auto const localizationMode = node.declare_parameter<std::string>("localization", "amcl");
        if (localizationMode != "amcl" and localizationMode != "truth")
            throw std::invalid_argument{"localization must be amcl or truth"};
        if (localizationMode == "amcl") {
            localization.open(node.declare_parameter<std::string>("localization_log", "localization.jsonl"));
            if (not localization)
                throw std::runtime_error{"cannot open localization telemetry"};
            localization << std::setprecision(12);
            estimate = node.create_subscription<geometry_msgs::msg::PoseWithCovarianceStamped>(
                "amcl_pose", rclcpp::QoS{10}.transient_local(),
                [this](geometry_msgs::msg::PoseWithCovarianceStamped const &message) { score(message); });
        }
        clock = node.create_publisher<rosgraph_msgs::msg::Clock>("clock", 10);
        odom = node.create_publisher<nav_msgs::msg::Odometry>("odom", 10);
        scan = node.create_publisher<sensor_msgs::msg::LaserScan>("scan", rclcpp::SensorDataQoS{});
        tf = node.create_publisher<tf2_msgs::msg::TFMessage>("tf", 100);
        staticTf =
            node.create_publisher<tf2_msgs::msg::TFMessage>("tf_static", rclcpp::QoS{1}.transient_local());
        map = node.create_publisher<nav_msgs::msg::OccupancyGrid>("map", rclcpp::QoS{1}.transient_local());
        velocity = node.create_subscription<geometry_msgs::msg::Twist>(
            "cmd_vel", 10, [this](geometry_msgs::msg::Twist const &message) {
                if (not std::isfinite(message.linear.x) or not std::isfinite(message.angular.z))
                    return;
                command = {message.linear.x, message.angular.z};
                receivedAt = std::chrono::steady_clock::now();
            });
        publishMap(node.declare_parameter<bool>("map_includes_obstacle", true));
        auto fixed = tf2_msgs::msg::TFMessage{};
        auto const zeroStamp = builtin_interfaces::msg::Time{};
        if (localizationMode == "truth")
            fixed.transforms.push_back(transform("map", "odom", zeroStamp));
        fixed.transforms.push_back(transform("base_link", "laser", zeroStamp));
        fixed.transforms.back().transform.translation.z = .2;
        staticTf->publish(fixed);
        timer = node.create_wall_timer(10ms, [this] { tick(); });
    }
    Bridge(Bridge const &) = delete;
    Bridge &operator=(Bridge const &) = delete;
    void stop() {
        timer->cancel();
        for (int i = 0; i < 250; ++i)
            physics->step({});
    }

  private:
    void score(geometry_msgs::msg::PoseWithCovarianceStamped const &message) {
        auto const stamp = rclcpp::Time{message.header.stamp}.nanoseconds();
        auto const sample = std::lower_bound(history.begin(), history.end(), stamp,
                                             [](auto const &entry, auto time) { return entry.first < time; });
        if (sample == history.end() or sample->first != stamp or message.header.frame_id != "map") {
            localization << "{\"matched\":false,\"stamp_ns\":" << stamp << "}" << std::endl;
            return;
        }
        auto const &truth = sample->second;
        auto const &pose = message.pose.pose;
        auto const &q = pose.orientation;
        auto const yaw = std::atan2(2 * (q.w * q.z + q.x * q.y), 1 - 2 * (q.y * q.y + q.z * q.z));
        auto const yawError = std::remainder(yaw - truth.yaw, 2 * std::acos(-1.0));
        localization << "{\"matched\":true,\"time\":" << truth.time << ",\"x\":" << pose.position.x
                     << ",\"y\":" << pose.position.y << ",\"yaw\":" << yaw << ",\"truth_x\":" << truth.x
                     << ",\"truth_y\":" << truth.y << ",\"truth_yaw\":" << truth.yaw
                     << ",\"xy_error\":" << std::hypot(pose.position.x - truth.x, pose.position.y - truth.y)
                     << ",\"yaw_error\":" << std::abs(yawError)
                     << ",\"variance_x\":" << message.pose.covariance[0]
                     << ",\"variance_y\":" << message.pose.covariance[7]
                     << ",\"variance_yaw\":" << message.pose.covariance[35] << "}" << std::endl;
    }
    void publishMap(bool const includeObstacle) {
        auto const source = physics->map(includeObstacle);
        auto message = nav_msgs::msg::OccupancyGrid{};
        message.header.frame_id = "map";
        message.info.resolution = static_cast<float>(source.resolution);
        message.info.width = source.width;
        message.info.height = source.height;
        message.info.origin.position.x = source.originX;
        message.info.origin.position.y = source.originY;
        message.info.origin.orientation.w = 1;
        message.data = source.cells;
        map->publish(message);
    }
    void tick() {
        auto effective = command;
        if (std::chrono::steady_clock::now() - receivedAt > 300ms)
            effective = {};
        for (int i = 0; i < 5; ++i)
            physics->step(effective);
        auto const state = physics->state();
        builtin_interfaces::msg::Time const stamp = rclcpp::Time{static_cast<std::int64_t>(state.time * 1e9)};
        history.emplace_back(rclcpp::Time{stamp}.nanoseconds(), state);
        if (history.size() > 2000)
            history.pop_front(); // 20 simulated seconds, never unbounded.
        auto time = rosgraph_msgs::msg::Clock{};
        time.clock = stamp;
        clock->publish(time);
        auto pose = transform("odom", "base_link", stamp);
        pose.transform.translation.x = state.x;
        pose.transform.translation.y = state.y;
        pose.transform.translation.z = state.z;
        pose.transform.rotation.x = state.orientation.x;
        pose.transform.rotation.y = state.orientation.y;
        pose.transform.rotation.z = state.orientation.z;
        pose.transform.rotation.w = state.orientation.w;
        tf->publish(tf2_msgs::msg::TFMessage{}.set__transforms({pose}));
        auto odometry = nav_msgs::msg::Odometry{};
        odometry.header = pose.header;
        odometry.child_frame_id = "base_link";
        odometry.pose.pose.position.x = state.x;
        odometry.pose.pose.position.y = state.y;
        odometry.pose.pose.position.z = state.z;
        odometry.pose.pose.orientation = pose.transform.rotation;
        odometry.twist.twist.linear.x = state.velocity.forward;
        odometry.twist.twist.angular.z = state.velocity.yawRate;
        odom->publish(odometry);
        if (++ticks % 10 == 0) {
            auto source = physics->scan();
            auto message = sensor_msgs::msg::LaserScan{};
            message.header.stamp = stamp;
            message.header.frame_id = "laser";
            message.angle_min = source.angleMin;
            message.angle_increment = source.angleStep;
            message.angle_max = source.angleMin + (source.ranges.size() - 1) * source.angleStep;
            message.range_min = .05f;
            message.range_max = source.rangeMax;
            message.scan_time = .1f;
            message.ranges = std::move(source.ranges);
            scan->publish(message);
        }
        if (viewer and not viewerClosed and ticks % 3 == 0)
            viewerClosed = not viewer->present(physics->scene());
        if (ticks % 20 == 0)
            telemetry << "{\"time\":" << state.time << ",\"x\":" << state.x << ",\"y\":" << state.y
                      << ",\"yaw\":" << state.yaw << ",\"v\":" << state.velocity.forward
                      << ",\"w\":" << state.velocity.yawRate << ",\"collision\":" << state.obstacleContact
                      << ",\"command_v\":" << effective.forward << ",\"viewer_closed\":" << viewerClosed
                      << "}" << std::endl;
    }
    rclcpp::Node &node;
    std::unique_ptr<simulation::Physics> physics;
    std::ofstream telemetry;
    std::ofstream localization;
    std::deque<std::pair<std::int64_t, simulation::State>> history;
    std::unique_ptr<simulation::Viewer> viewer;
    bool viewerClosed = false;
    simulation::Velocity command;
    std::chrono::steady_clock::time_point receivedAt{};
    int ticks = 0;
    rclcpp::Publisher<rosgraph_msgs::msg::Clock>::SharedPtr clock;
    rclcpp::Publisher<nav_msgs::msg::Odometry>::SharedPtr odom;
    rclcpp::Publisher<sensor_msgs::msg::LaserScan>::SharedPtr scan;
    rclcpp::Publisher<nav_msgs::msg::OccupancyGrid>::SharedPtr map;
    rclcpp::Publisher<tf2_msgs::msg::TFMessage>::SharedPtr tf, staticTf;
    rclcpp::Subscription<geometry_msgs::msg::Twist>::SharedPtr velocity;
    rclcpp::Subscription<geometry_msgs::msg::PoseWithCovarianceStamped>::SharedPtr estimate;
    rclcpp::TimerBase::SharedPtr timer;
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
        auto node = std::make_shared<rclcpp::Node>("mujoco_bridge");
        auto const model = node->declare_parameter<std::string>("model");
        auto physics = simulation::makePhysics(model);
        auto viewer = std::unique_ptr<simulation::Viewer>{};
        auto const view = node->declare_parameter<bool>("viewer", false);
        auto const mode = node->declare_parameter<std::string>("executor", "single");
        if (view and mode != "single")
            throw std::invalid_argument{"viewer requires the main-thread single executor"};
        if (view)
            viewer =
                simulation::makeViewer({model, node->declare_parameter<std::string>("viewer_capture", ""),
                                        node->declare_parameter<double>("viewer_target_x", 4.0),
                                        node->declare_parameter<double>("viewer_target_y", 0.0)});
        auto bridge = Bridge{*node, std::move(physics), std::move(viewer)};
        std::unique_ptr<rclcpp::Executor> executor;
        if (mode == "single")
            executor = std::make_unique<rclcpp::executors::SingleThreadedExecutor>();
        else if (mode == "multi")
            executor =
                std::make_unique<rclcpp::executors::MultiThreadedExecutor>(rclcpp::ExecutorOptions{}, 4);
        else
            throw std::invalid_argument{"executor must be single or multi"};
        executor->add_node(node);
        auto cancel = lexec::inplace_stop_callback{stop.get_token(), [&]() noexcept { executor->cancel(); }};
        // rclcpp cancel() before spin() is not sticky; cover the narrow startup race.
        auto shutdownPoll = node->create_wall_timer(10ms, [&] {
            if (stop.stop_requested())
                executor->cancel();
        });
        if (not stop.stop_requested())
            executor->spin();
        bridge.stop();
        status = signals.error() ? 1 : 0;
    } catch (std::exception const &error) {
        std::cerr << error.what() << '\n';
        status = 1;
    }
    rclcpp::shutdown();
    return status;
}
