#include "Physics.h"
#include <cmath>
#include <iostream>
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
        std::cout << "MuJoCo wheel physics and laser smoke test passed\n";
        return 0;
    } catch (std::exception const &error) {
        std::cerr << error.what() << '\n';
        return 1;
    }
}
