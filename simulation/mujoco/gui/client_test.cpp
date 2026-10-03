#include "GoalInbox.h"
#include "GuiClient.h"
#include "SceneProtocol.h"
#include <chrono>
#include <geometry_msgs/msg/pose_array.hpp>
#include <iostream>
#include <rclcpp/rclcpp.hpp>
#include <std_msgs/msg/string.hpp>
#include <std_srvs/srv/trigger.hpp>
#include <stdexcept>
#include <thread>

using namespace std::chrono_literals;
namespace {
void check(bool value, char const *message) {
    if (not value)
        throw std::runtime_error{message};
}
} // namespace
int main(int argc, char **argv) {
    rclcpp::init(argc, argv);
    auto result = 0;
    try {
        auto server = std::make_shared<rclcpp::Node>("gui_protocol_server");
        auto client = simulation::makeGuiClient(std::make_shared<rclcpp::Node>("gui_protocol_client"));
        auto executor = rclcpp::executors::SingleThreadedExecutor{};
        executor.add_node(server);
        auto const latest = rclcpp::QoS{1}.transient_local();
        auto scenePublisher = server->create_publisher<std_msgs::msg::String>("simulation/scene", latest);
        auto statusPublisher = server->create_publisher<std_msgs::msg::String>("navigation/status", latest);
        auto routeResult = server->create_publisher<std_msgs::msg::String>("navigation/route_result", 16);
        auto editResult = server->create_publisher<std_msgs::msg::String>("simulation/edit_result", 32);
        std::optional<geometry_msgs::msg::PoseArray> route;
        std::optional<visualization_msgs::msg::Marker> edit;
        auto routes = server->create_subscription<geometry_msgs::msg::PoseArray>(
            "navigation/route", 1, [&](geometry_msgs::msg::PoseArray const &message) { route = message; });
        auto edits = server->create_subscription<visualization_msgs::msg::Marker>(
            "simulation/edit_obstacle", 1,
            [&](visualization_msgs::msg::Marker const &message) { edit = message; });
        auto scene = simulation::SceneState{std::vector<double>(9), 0, std::vector<double>(48), 0};
        scene.positions[3] = 1;
        auto status = simulation::SessionStatus{};
        status.ready = true;
        auto publish = [](auto const &publisher, std::string text) {
            auto message = std_msgs::msg::String{};
            message.data = std::move(text);
            publisher->publish(message);
        };
        auto tick = [&] {
            publish(scenePublisher, simulation::sceneJson(scene));
            publish(statusPublisher, simulation::statusJson(status));
            executor.spin_some();
            client->poll();
            std::this_thread::sleep_for(10ms);
        };
        auto wait = [&](auto predicate, char const *message) {
            auto const deadline = std::chrono::steady_clock::now() + 3s;
            while (not predicate() and std::chrono::steady_clock::now() < deadline)
                tick();
            check(predicate(), message);
        };
        wait(
            [&] {
                return client->state().connected and routes->get_publisher_count() > 0 and
                       routeResult->get_subscription_count() > 0;
            },
            "protocol discovery failed");
        check(client->sendRoute({{1, 0, 0}}), "route not sent");
        wait([&] { return route.has_value(); }, "route not delivered");
        status.accepted = 1;
        status.active = 1;
        for (int i = 0; i < 10; ++i)
            tick();
        check(client->state().routePending, "unrelated global counter acknowledged Qt route");
        auto const stamp = route->header.stamp;
        auto ack = [&](int sec) {
            publish(routeResult, "{\"request_sec\":" + std::to_string(sec) +
                                     ",\"request_nanosec\":" + std::to_string(stamp.nanosec) +
                                     ",\"accepted\":true,\"status\":" + simulation::statusJson(status) + "}");
        };
        ack(stamp.sec - 1);
        for (int i = 0; i < 10; ++i)
            tick();
        check(client->state().routePending, "unrelated request acknowledged Qt route");
        status.accepted = 2;
        status.active = 2;
        ack(stamp.sec);
        wait([&] { return not client->state().routePending; }, "matching route ACK ignored");
        client->cancel();
        status.active = 0;
        status.lastResult = "succeeded";
        for (int i = 0; i < 10; ++i)
            tick();
        check(client->state().cancelPending and not client->sendRoute({{0, 0, 0}}),
              "old cancel intent allowed a new route");
        auto canceled = false;
        auto cancel = server->create_service<std_srvs::srv::Trigger>(
            "navigation/cancel", [&](std_srvs::srv::Trigger::Request::SharedPtr,
                                     std_srvs::srv::Trigger::Response::SharedPtr response) {
                canceled = true;
                response->success = true;
            });
        wait([&] { return canceled and not client->state().cancelPending; },
             "cancel intent lost before service discovery");
        check(client->edit({0, true, 1, 1}), "edit not sent");
        wait([&] { return edit.has_value(); }, "edit not delivered");
        // Scene arrives first; ACK must still complete on a subsequent snapshot.
        scene.revision = 1;
        for (int i = 0; i < 10; ++i)
            tick();
        check(client->state().editPending, "scene alone acknowledged edit");
        publish(editResult, simulation::editJson(*edit, {true, 1, {}}));
        wait([&] { return not client->state().editPending; }, "scene-before-ACK edit did not complete");
        check(client->state().editApplied, "successful edit marked failed");
        edit.reset();
        check(client->edit({0, false}), "delete not sent");
        wait([&] { return edit.has_value(); }, "delete not delivered");
        publish(editResult, simulation::editJson(*edit, {true, 2, {}}));
        for (int i = 0; i < 10; ++i)
            tick();
        check(client->state().editPending, "ACK before scene prematurely completed edit");
        scene.revision = 2;
        wait([&] { return not client->state().editPending; }, "ACK-before-scene edit did not complete");
        std::cout << "GUI request correlation, delayed cancel service, edit ACK ordering passed\n";
    } catch (std::exception const &error) {
        std::cerr << error.what() << '\n';
        result = 1;
    }
    rclcpp::shutdown();
    return result;
}
