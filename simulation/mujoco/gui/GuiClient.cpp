#include "GuiClient.h"
#include <QJsonArray>
#include <QJsonDocument>
#include <QJsonObject>
#include <QUuid>
#include <algorithm>
#include <chrono>
#include <cmath>
#include <geometry_msgs/msg/pose_array.hpp>
#include <nav_msgs/msg/path.hpp>
#include <rclcpp/rclcpp.hpp>
#include <std_msgs/msg/string.hpp>
#include <std_srvs/srv/trigger.hpp>
#include <visualization_msgs/msg/marker.hpp>

namespace simulation {
namespace {
using Clock = std::chrono::steady_clock;
using namespace std::chrono_literals;
QJsonObject object(std::string const &text) {
    return QJsonDocument::fromJson(QByteArray::fromStdString(text)).object();
}
bool counter(QJsonValue const &value) {
    auto const n = value.toDouble(-1);
    return value.isDouble() and std::isfinite(n) and n >= 0 and n <= 9007199254740991.0 and
           std::floor(n) == n;
}
bool finitePose(geometry_msgs::msg::Pose const &pose) {
    return std::isfinite(pose.position.x) and std::isfinite(pose.position.y) and
           std::isfinite(pose.orientation.z) and std::isfinite(pose.orientation.w);
}
std::vector<double> numbers(QJsonValue const &value) {
    auto result = std::vector<double>{};
    for (auto const item : value.toArray()) {
        if (not item.isDouble() or not std::isfinite(item.toDouble()))
            return {};
        result.push_back(item.toDouble());
    }
    return result;
}
Waypoint waypoint(geometry_msgs::msg::Pose const &pose) {
    return {pose.position.x, pose.position.y, 2 * std::atan2(pose.orientation.z, pose.orientation.w)};
}
struct GuiClientImpl final : GuiClient {
    explicit GuiClientImpl(std::shared_ptr<rclcpp::Node> node_) : node{std::move(node_)} {
        executor.add_node(node);
        routePublisher = node->create_publisher<geometry_msgs::msg::PoseArray>("navigation/route", 1);
        editPublisher =
            node->create_publisher<visualization_msgs::msg::Marker>("simulation/edit_obstacle", 1);
        cancelClient = node->create_client<std_srvs::srv::Trigger>("navigation/cancel");
        auto const latest = rclcpp::QoS{1}.transient_local();
        sceneSubscription = node->create_subscription<std_msgs::msg::String>(
            "simulation/scene", latest,
            [this](std_msgs::msg::String const &message) { observeScene(message.data); });
        statusSubscription = node->create_subscription<std_msgs::msg::String>(
            "navigation/status", latest,
            [this](std_msgs::msg::String const &message) { observeStatus(message.data); });
        routeSubscription = node->create_subscription<geometry_msgs::msg::PoseArray>(
            "navigation/active_route", latest, [this](geometry_msgs::msg::PoseArray const &message) {
                if (message.header.frame_id != "map" or message.poses.size() > 64 or
                    not std::all_of(message.poses.begin(), message.poses.end(), finitePose))
                    return;
                snapshot.activeRoute.clear();
                for (auto const &pose : message.poses)
                    snapshot.activeRoute.push_back(waypoint(pose));
                ++snapshot.routeRevision;
            });
        pathSubscription = node->create_subscription<nav_msgs::msg::Path>(
            "navigation/path", latest, [this](nav_msgs::msg::Path const &message) {
                if (message.header.frame_id != "map" or message.poses.size() > 10000 or
                    not std::all_of(message.poses.begin(), message.poses.end(),
                                    [](auto const &pose) { return finitePose(pose.pose); }))
                    return;
                snapshot.path.clear();
                for (auto const &pose : message.poses)
                    snapshot.path.push_back(waypoint(pose.pose));
            });
        resultSubscription = node->create_subscription<std_msgs::msg::String>(
            "simulation/edit_result", 32,
            [this](std_msgs::msg::String const &message) { observeEdit(message.data); });
        routeResultSubscription = node->create_subscription<std_msgs::msg::String>(
            "navigation/route_result", 16, [this](std_msgs::msg::String const &message) {
                auto const json = object(message.data);
                if (not snapshot.routePending or json["request_sec"].toInt(-1) != routeRequest.sec or
                    json["request_nanosec"].toInt(-1) != static_cast<int>(routeRequest.nanosec) or
                    not json["accepted"].isBool())
                    return;
                observeStatus(QJsonDocument{json["status"].toObject()}.toJson().toStdString());
                snapshot.routePending = false;
                snapshot.feedback =
                    json["accepted"].toBool() ? "任务已接受" : "任务被拒绝：" + snapshot.navigation.message;
            });
    }
    void observeScene(std::string const &text) {
        auto const json = object(text);
        auto qpos = numbers(json["qpos"]), mocap = numbers(json["mocap_pos"]);
        if (json["version"].toInt() != 1 or qpos.size() != 9 or mocap.size() != 3 * obstacleCapacity or
            not counter(json["revision"]) or not json["time"].isDouble() or
            not std::isfinite(json["time"].toDouble()))
            return;
        snapshot.scene = {std::move(qpos), json["time"].toDouble(), std::move(mocap),
                          static_cast<std::uint64_t>(json["revision"].toDouble())};
        lastScene = Clock::now();
        if (confirmedRevision and snapshot.scene.revision >= *confirmedRevision) {
            confirmedRevision.reset();
            snapshot.editPending = false;
            ++snapshot.editResponses;
            snapshot.editApplied = true;
            snapshot.feedback = "障碍物已应用";
        }
    }
    void observeStatus(std::string const &text) {
        auto const json = object(text);
        if (not json["state"].isString())
            return;
        for (auto const *key : {"active", "pending", "accepted", "rejected", "last_id", "waypoint_index",
                                "waypoint_count", "waypoint_completed"})
            if (not counter(json[key]))
                return;
        if (json["waypoint_count"].toDouble() > 64 or
            json["waypoint_index"].toDouble() > json["waypoint_count"].toDouble() or
            json["waypoint_completed"].toDouble() > json["waypoint_count"].toDouble())
            return;
        if (json["accepted"].toDouble() < static_cast<double>(snapshot.navigation.accepted))
            return;
        auto &status = snapshot.navigation;
        status.phase = json["state"].toString().toStdString();
        status.result = json["last_result"].toString().toStdString();
        status.message = json["message"].toString().toStdString();
        status.active = static_cast<std::uint64_t>(json["active"].toDouble());
        status.pending = static_cast<std::uint64_t>(json["pending"].toDouble());
        status.accepted = static_cast<std::uint64_t>(json["accepted"].toDouble());
        status.rejected = static_cast<std::uint64_t>(json["rejected"].toDouble());
        status.lastId = static_cast<std::uint64_t>(json["last_id"].toDouble());
        status.index = static_cast<std::size_t>(json["waypoint_index"].toDouble());
        status.count = static_cast<std::size_t>(json["waypoint_count"].toDouble());
        status.completed = static_cast<std::size_t>(json["waypoint_completed"].toDouble());
        lastStatus = Clock::now();
    }
    void observeEdit(std::string const &text) {
        auto const json = object(text);
        if (not snapshot.editPending or json["client"].toString() != clientId or
            json["request_nanosec"].toInt() != requestId or json["request_sec"].toInt() != 0)
            return;
        if (not json["applied"].isBool() or not counter(json["revision"]))
            return;
        if (json["applied"].toBool()) {
            confirmedRevision = static_cast<std::uint64_t>(json["revision"].toDouble());
            snapshot.feedback = "物理端已确认，等待场景同步";
        } else {
            snapshot.editPending = false;
            ++snapshot.editResponses;
            snapshot.editApplied = false;
            snapshot.feedback = "障碍物未应用：" + json["reason"].toString().toStdString();
        }
    }
    void poll() override {
        executor.spin_some(2ms);
        auto const now = Clock::now();
        snapshot.connected = now - lastScene < 2s and now - lastStatus < 2s;
        if (snapshot.editPending and now - editSent > 4s) {
            snapshot.editPending = false;
            confirmedRevision.reset();
            ++snapshot.editResponses;
            snapshot.editApplied = false;
            snapshot.feedback = "障碍物确认超时，请检查场景后重试";
        }
        if (snapshot.routePending and now - routeSent > 4s) {
            snapshot.routePending = false;
            snapshot.feedback = "任务确认超时，请检查连接";
        }
        if (cancelWanted) {
            if (now - cancelWantedAt > 8s) {
                cancelWanted = false;
                snapshot.cancelPending = false;
                snapshot.feedback = "取消服务未就绪，请重试停止任务";
            } else if (not snapshot.routePending and cancelClient->service_is_ready()) {
                sendCancel();
                cancelWanted = false;
            }
        }
        if (cancelRequest and now - cancelSent > 4s) {
            cancelClient->remove_pending_request(*cancelRequest);
            cancelRequest.reset();
            snapshot.cancelPending = false;
            snapshot.feedback = "取消确认超时，请重试";
        }
    }
    GuiState const &state() const override { return snapshot; }
    bool sendRoute(std::vector<Waypoint> const &route) override {
        if (not snapshot.connected or snapshot.routePending or snapshot.cancelPending or
            snapshot.navigation.pending or snapshot.navigation.phase != "ready" or route.empty() or
            route.size() > 64 or routePublisher->get_subscription_count() == 0)
            return false;
        auto message = geometry_msgs::msg::PoseArray{};
        message.header.frame_id = "map";
        routeRequest = node->now();
        message.header.stamp = routeRequest;
        for (auto const &point : route) {
            auto pose = geometry_msgs::msg::Pose{};
            pose.position.x = point.x;
            pose.position.y = point.y;
            pose.orientation.z = std::sin(point.yaw / 2);
            pose.orientation.w = std::cos(point.yaw / 2);
            message.poses.push_back(pose);
        }
        snapshot.routePending = true;
        routeSent = Clock::now();
        routePublisher->publish(message);
        return true;
    }
    void cancel() override {
        if (cancelWanted or cancelRequest)
            return;
        cancelWanted = true;
        snapshot.cancelPending = true;
        cancelWantedAt = Clock::now();
        snapshot.feedback = "正在请求停止任务";
    }
    void sendCancel() {
        auto future = cancelClient->async_send_request(
            std::make_shared<std_srvs::srv::Trigger::Request>(),
            [this](rclcpp::Client<std_srvs::srv::Trigger>::SharedFuture result) {
                snapshot.feedback = result.get()->success ? "正在取消，等待任务结束" : "当前无任务";
                cancelRequest.reset();
                snapshot.cancelPending = false;
            });
        cancelRequest = future.request_id;
        cancelSent = Clock::now();
    }
    bool edit(ObstacleEdit const &edit) override {
        if (not snapshot.connected or snapshot.editPending or editPublisher->get_subscription_count() == 0)
            return false;
        using Marker = visualization_msgs::msg::Marker;
        auto message = Marker{};
        message.header.frame_id = "map";
        requestId = requestId % 999999998 + 1;
        message.header.stamp.nanosec = static_cast<unsigned>(requestId);
        message.ns = clientId.toStdString();
        message.id = static_cast<int>(edit.slot);
        message.type = Marker::CUBE;
        message.action = edit.active ? Marker::ADD : Marker::DELETE;
        message.pose.position.x = edit.x;
        message.pose.position.y = edit.y;
        message.pose.position.z = obstacleHeight / 2;
        message.pose.orientation.w = 1;
        message.scale.x = obstacleLength;
        message.scale.y = obstacleWidth;
        message.scale.z = obstacleHeight;
        snapshot.editPending = true;
        snapshot.feedback = "正在应用障碍物";
        editSent = Clock::now();
        editPublisher->publish(message);
        return true;
    }
    std::shared_ptr<rclcpp::Node> node;
    rclcpp::executors::SingleThreadedExecutor executor;
    GuiState snapshot;
    QString clientId = QUuid::createUuid().toString(QUuid::WithoutBraces);
    int requestId = 0;
    bool cancelWanted = false;
    builtin_interfaces::msg::Time routeRequest;
    std::optional<std::uint64_t> confirmedRevision;
    std::optional<std::int64_t> cancelRequest;
    Clock::time_point lastScene{}, lastStatus{}, routeSent{}, editSent{}, cancelSent{}, cancelWantedAt{};
    rclcpp::Publisher<geometry_msgs::msg::PoseArray>::SharedPtr routePublisher;
    rclcpp::Publisher<visualization_msgs::msg::Marker>::SharedPtr editPublisher;
    rclcpp::Subscription<std_msgs::msg::String>::SharedPtr sceneSubscription, statusSubscription,
        resultSubscription, routeResultSubscription;
    rclcpp::Subscription<geometry_msgs::msg::PoseArray>::SharedPtr routeSubscription;
    rclcpp::Subscription<nav_msgs::msg::Path>::SharedPtr pathSubscription;
    rclcpp::Client<std_srvs::srv::Trigger>::SharedPtr cancelClient;
};
} // namespace
std::unique_ptr<GuiClient> makeGuiClient(std::shared_ptr<rclcpp::Node> node) {
    return std::make_unique<GuiClientImpl>(std::move(node));
}
} // namespace simulation
