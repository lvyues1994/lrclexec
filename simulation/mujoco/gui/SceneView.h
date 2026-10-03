#pragma once

#include "GuiClient.h"
#include <QOpenGLWindow>
#include <functional>
#include <memory>

namespace simulation {
struct SceneOverlay {
    std::vector<Waypoint> points, path;
    std::optional<Waypoint> pointPreview;
    std::optional<ObstacleEdit> boxPreview;
    int selectedPoint = -1;
};
struct ScenePick {
    double x, y;
    std::optional<std::size_t> obstacle;
    std::optional<std::size_t> waypoint;
};
struct SceneView final : QOpenGLWindow {
    explicit SceneView(std::string const &file, std::function<void(ScenePick const &)> pick);
    ~SceneView() override;
    void present(SceneState const &state, SceneOverlay const &overlay);
    void topView(bool enabled);

  protected:
    void initializeGL() override;
    void paintGL() override;
    void resizeGL(int width, int height) override;
    void mousePressEvent(QMouseEvent *event) override;
    void mouseMoveEvent(QMouseEvent *event) override;
    void mouseReleaseEvent(QMouseEvent *event) override;
    void wheelEvent(QWheelEvent *event) override;

  private:
    struct RenderState;
    std::unique_ptr<RenderState> render;
};
} // namespace simulation
