#pragma once

#include "Physics.h"

namespace simulation {
struct ViewerConfig {
    std::string modelFile;
    std::string captureFile;
    double targetX = 4;
    double targetY = 0;
    bool hasTarget = true;
};
struct Viewer {
    virtual ~Viewer() = default;
    virtual bool present(SceneState const &state) = 0;
    virtual void setGoal(double x, double y) = 0;
};
std::unique_ptr<Viewer> makeViewer(ViewerConfig const &config);
} // namespace simulation
