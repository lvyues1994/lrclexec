#pragma once

#include "Physics.h"
#include <memory>
#include <optional>
#include <string>
#include <vector>

namespace rclcpp {
class Node;
}
namespace simulation {
struct Waypoint {
    double x = 0, y = 0, yaw = 0;
};
struct NavigationState {
    std::string phase = "starting", result, message;
    std::uint64_t active = 0, pending = 0, accepted = 0, rejected = 0, lastId = 0;
    std::size_t index = 0, count = 0, completed = 0;
};
struct GuiState {
    SceneState scene;
    NavigationState navigation;
    std::vector<Waypoint> activeRoute, path;
    std::uint64_t routeRevision = 0, editResponses = 0;
    bool connected = false, routePending = false, editPending = false, cancelPending = false;
    bool editApplied = false;
    std::string feedback;
};
// poll and all commands are called on the Qt thread. Physics lives in bridge.
struct GuiClient {
    virtual ~GuiClient() = default;
    virtual void poll() = 0;
    virtual GuiState const &state() const = 0;
    virtual bool sendRoute(std::vector<Waypoint> const &route) = 0;
    virtual void cancel() = 0;
    virtual bool edit(ObstacleEdit const &edit) = 0;
};
std::unique_ptr<GuiClient> makeGuiClient(std::shared_ptr<rclcpp::Node> node);
} // namespace simulation
