#pragma once

#include <array>
#include <cstdint>
#include <memory>
#include <string>
#include <vector>

namespace simulation {
inline constexpr std::size_t obstacleCapacity = 16;
inline constexpr double obstacleLength = .8, obstacleWidth = .6, obstacleHeight = .6;
struct ObstacleEdit {
    std::size_t slot;
    bool active;
    double x = 0, y = 0;
};
struct EditResult {
    bool applied;
    std::uint64_t revision;
    std::string reason;
};

struct Velocity {
    double forward = 0;
    double yawRate = 0;
};
struct Orientation {
    double w;
    double x;
    double y;
    double z;
};
struct State {
    double time;
    double x;
    double y;
    double yaw;
    double z;
    Orientation orientation;
    Velocity velocity;
    bool obstacleContact;
};
struct Scan {
    std::vector<float> ranges;
    float angleMin;
    float angleStep;
    float rangeMax;
};
struct Map {
    std::vector<std::int8_t> cells;
    unsigned width;
    unsigned height;
    double resolution;
    double originX;
    double originY;
};
struct SceneState {
    std::vector<double> positions;
    double time;
    std::vector<double> mocapPositions;
    std::uint64_t revision = 0;
};
struct Physics {
    virtual ~Physics() = default;
    virtual void step(Velocity command) = 0;
    virtual State state() const = 0;
    virtual Scan scan() const = 0;
    virtual Map map(bool includeObstacle) const = 0;
    virtual SceneState scene() const = 0;
    virtual EditResult editObstacle(ObstacleEdit const &edit) = 0;
};
std::unique_ptr<Physics> makePhysics(std::string const &modelFile);

} // namespace simulation
