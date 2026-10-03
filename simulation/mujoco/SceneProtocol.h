#pragma once

#include "Physics.h"
#include <optional>
#include <visualization_msgs/msg/marker.hpp>

namespace simulation {
struct ParsedEdit {
    std::optional<ObstacleEdit> edit;
    std::string reason;
};
ParsedEdit parseEdit(visualization_msgs::msg::Marker const &message);
std::string editJson(visualization_msgs::msg::Marker const &request, EditResult const &result);
std::string sceneJson(SceneState const &state);
} // namespace simulation
