#include "Physics.h"
#include <cmath>
#include <iostream>
#include <limits>
#include <stdexcept>

void require(bool const value, char const *message) {
    if (not value)
        throw std::runtime_error{message};
}
int main(int argc, char **argv) {
    try {
        if (argc != 3)
            throw std::invalid_argument{"provide normal and tilted-laser model paths"};
        auto tilted = simulation::makePhysics(argv[2]);
        for (int i = 0; i < 250; ++i)
            tilted->step({});
        require(std::abs(tilted->scan().ranges[180] - .6f) < .03f,
                "tilted laser did not preserve pitch and hit the ground");
        auto physics = simulation::makePhysics(argv[1]);
        for (int i = 0; i < 250; ++i)
            physics->step({});
        auto scan = physics->scan();
        require(std::abs(scan.ranges[180] - 1.7f) < .03f, "forward laser did not hit obstacle");
        for (int i = 0; i < 1000; ++i)
            physics->step({.2, 0});
        auto state = physics->state();
        std::cout << "straight x=" << state.x << " y=" << state.y << " v=" << state.velocity.forward
                  << " yaw=" << state.yaw << '\n';
        require(state.x > .3 and state.x < .5 and std::abs(state.y) < .02, "wheels did not drive forward");
        for (int i = 0; i < 1000; ++i)
            physics->step({0, .5});
        state = physics->state();
        require(state.yaw > .8 and state.yaw < 1.2, "positive yaw command turned the wrong way");
        for (int i = 0; i < 500; ++i)
            physics->step({});
        state = physics->state();
        require(std::abs(state.velocity.forward) < .01 and std::abs(state.velocity.yawRate) < .02,
                "zero command did not stop robot");
        require(not state.obstacleContact, "open loop unexpectedly collided");
        auto dynamic = simulation::makePhysics(argv[1]);
        for (int i = 0; i < 250; ++i)
            dynamic->step({});
        auto const before = dynamic->scene();
        auto const originalMap = dynamic->map(true).cells;
        auto const wallMap = dynamic->map(false).cells;
        require(dynamic->editObstacle({0, true, 1, 0}).applied, "dynamic box add rejected");
        require(std::abs(dynamic->scan().ranges[180] - .6f) < .03f, "laser missed new box");
        require(dynamic->scene().time == before.time and dynamic->scene().positions == before.positions,
                "obstacle edit reset robot state or simulation clock");
        require(dynamic->map(true).cells == originalMap and dynamic->map(false).cells == wallMap,
                "dynamic box changed static occupancy map");
        auto const revision = dynamic->scene().revision;
        require(not dynamic->editObstacle({1, true, 1, 0}).applied, "overlapping boxes accepted");
        require(not dynamic->editObstacle({0, true, 0, 0}).applied, "box overlapped robot");
        require(not dynamic->editObstacle({16, true, 1, 1}).applied, "invalid slot accepted");
        require(not dynamic->editObstacle({0, true, 6, 0}).applied, "wall overlap accepted");
        require(not dynamic->editObstacle({0, true, std::numeric_limits<double>::quiet_NaN(), 1}).applied,
                "non-finite obstacle accepted");
        require(dynamic->scene().revision == revision, "rejection changed scene revision");
        require(dynamic->editObstacle({0, true, 1, 1}).applied, "move rejected");
        require(std::abs(dynamic->scan().ranges[180] - 1.7f) < .03f, "moving box did not restore ray");
        require(dynamic->editObstacle({15, true, -1, -1}).applied, "last slot unusable");
        require(dynamic->editObstacle({0, false}).applied and dynamic->scene().mocapPositions[2] < 0,
                "deleting box did not park it");
        require(dynamic->editObstacle({0, true, 1, 0}).applied, "reusing deleted slot failed");
        for (int i = 0; i < 3000; ++i)
            dynamic->step({.3, 0});
        require(dynamic->state().obstacleContact and dynamic->state().x < .6,
                "dynamic box did not physically stop the robot");
        std::cout << "MuJoCo wheel physics and laser smoke test passed\n";
        return 0;
    } catch (std::exception const &error) {
        std::cerr << error.what() << '\n';
        return 1;
    }
}
