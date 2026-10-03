#include "SceneProtocol.h"
#include <cmath>
#include <iomanip>
#include <sstream>

namespace simulation {
ParsedEdit parseEdit(visualization_msgs::msg::Marker const &message) {
    using Marker = visualization_msgs::msg::Marker;
    if (message.header.frame_id != "map" or message.id < 0 or
        static_cast<std::size_t>(message.id) >= obstacleCapacity)
        return {{}, "Expected map frame and a slot in 0..15"};
    if (message.action == Marker::DELETE)
        return {ObstacleEdit{static_cast<std::size_t>(message.id), false}, {}};
    if (message.action != Marker::ADD or message.type != Marker::CUBE)
        return {{}, "Expected CUBE ADD or DELETE"};
    auto const near = [](double value, double expected) {
        return std::isfinite(value) and std::abs(value - expected) < 1e-6;
    };
    auto const &p = message.pose.position;
    auto const &q = message.pose.orientation;
    if (not std::isfinite(p.x) or not std::isfinite(p.y) or not near(p.z, obstacleHeight / 2) or
        not near(q.x, 0) or not near(q.y, 0) or not near(q.z, 0) or not near(std::abs(q.w), 1) or
        not near(message.scale.x, obstacleLength) or not near(message.scale.y, obstacleWidth) or
        not near(message.scale.z, obstacleHeight))
        return {{}, "Expected finite position and an axis-aligned 0.8 x 0.6 x 0.6 m box"};
    return {ObstacleEdit{static_cast<std::size_t>(message.id), true, p.x, p.y}, {}};
}
std::string editJson(visualization_msgs::msg::Marker const &request, EditResult const &result) {
    auto client = request.ns;
    for (auto &c : client)
        if (static_cast<unsigned char>(c) < 32)
            c = '?';
    auto text = std::ostringstream{};
    text << "{\"client\":" << std::quoted(client) << ",\"request_sec\":" << request.header.stamp.sec
         << ",\"request_nanosec\":" << request.header.stamp.nanosec << ",\"slot\":" << request.id
         << ",\"applied\":" << (result.applied ? "true" : "false") << ",\"revision\":" << result.revision
         << ",\"reason\":" << std::quoted(result.reason) << '}';
    return text.str();
}
std::string sceneJson(SceneState const &state) {
    auto text = std::ostringstream{};
    text << std::setprecision(15) << "{\"version\":1,\"time\":" << state.time
         << ",\"revision\":" << state.revision;
    auto array = [&](char const *name, std::vector<double> const &values) {
        text << ",\"" << name << "\":[";
        for (std::size_t i = 0; i < values.size(); ++i) {
            if (i)
                text << ',';
            text << values[i];
        }
        text << ']';
    };
    array("qpos", state.positions);
    array("mocap_pos", state.mocapPositions);
    text << '}';
    return text.str();
}
} // namespace simulation
