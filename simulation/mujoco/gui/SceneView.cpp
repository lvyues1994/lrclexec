#include "SceneView.h"
#include <QMouseEvent>
#include <QSurfaceFormat>
#include <QWheelEvent>
#include <algorithm>
#include <array>
#include <cmath>
#include <cstdio>
#include <mujoco/mujoco.h>
#include <stdexcept>

namespace simulation {
namespace {
struct ModelDeleter {
    void operator()(mjModel *p) const noexcept { mj_deleteModel(p); }
};
struct DataDeleter {
    void operator()(mjData *p) const noexcept { mj_deleteData(p); }
};
using Model = std::unique_ptr<mjModel, ModelDeleter>;
using Data = std::unique_ptr<mjData, DataDeleter>;
Model load(std::string const &file) {
    auto error = std::array<char, 1024>{};
    auto model = Model{mj_loadXML(file.c_str(), nullptr, error.data(), static_cast<int>(error.size()))};
    if (not model)
        throw std::runtime_error{error.data()};
    return model;
}
struct SceneResource {
    mjvScene value{};
    explicit SceneResource(mjModel const *model) {
        mjv_defaultScene(&value);
        mjv_makeScene(model, &value, 4096);
    }
    ~SceneResource() { mjv_freeScene(&value); }
    SceneResource(SceneResource const &) = delete;
    SceneResource &operator=(SceneResource const &) = delete;
};
struct ContextResource {
    mjrContext value{};
    explicit ContextResource(mjModel const *model) {
        mjr_defaultContext(&value);
        mjr_makeContext(model, &value, mjFONTSCALE_150);
    }
    ~ContextResource() { mjr_freeContext(&value); }
    ContextResource(ContextResource const &) = delete;
    ContextResource &operator=(ContextResource const &) = delete;
};
using Color = std::array<float, 4>;
constexpr Color pointColor{.2f, .5f, 1.f, 1.f}, pathColor{.15f, .85f, .4f, 1.f};
} // namespace
struct SceneView::RenderState {
    explicit RenderState(std::string const &file, std::function<void(ScenePick const &)> pick_)
        : model{load(file)}, data{mj_makeData(model.get())}, scene{model.get()}, pick{std::move(pick_)} {
        if (not data)
            throw std::bad_alloc{};
        mjv_defaultCamera(&camera);
        mjv_defaultOption(&options);
        options.geomgroup[3] = 1;
        options.label = mjLABEL_SELECTION;
        camera.lookat[0] = 2;
        camera.distance = 10;
        camera.azimuth = 90;
        camera.elevation = -65;
        mj_forward(model.get(), data.get());
    }
    void sphere(Waypoint const &p, Color const &color, double radius, int label = 0) {
        if (scene.value.ngeom == scene.value.maxgeom)
            return;
        auto const size = std::array<mjtNum, 3>{radius, radius, .04};
        auto const pos = std::array<mjtNum, 3>{p.x, p.y, .045};
        auto &geom = scene.value.geoms[scene.value.ngeom++];
        mjv_initGeom(&geom, mjGEOM_CYLINDER, size.data(), pos.data(), nullptr, color.data());
        if (label)
            std::snprintf(geom.label, sizeof geom.label, "%d", label);
    }
    void line(Waypoint const &a, Waypoint const &b, Color const &color, double width,
              int type = mjGEOM_LINE) {
        if (scene.value.ngeom == scene.value.maxgeom)
            return;
        auto &geom = scene.value.geoms[scene.value.ngeom++];
        mjv_initGeom(&geom, type, nullptr, nullptr, nullptr, color.data());
        auto const from = std::array<mjtNum, 3>{a.x, a.y, .065}, to = std::array<mjtNum, 3>{b.x, b.y, .065};
        mjv_connector(&geom, type, width, from.data(), to.data());
    }
    void decorate() {
        for (std::size_t i = 1; i < overlay.path.size(); ++i)
            line(overlay.path[i - 1], overlay.path[i], pathColor, 4);
        for (std::size_t i = 0; i < overlay.points.size(); ++i) {
            auto const &p = overlay.points[i];
            if (i) {
                auto const &a = overlay.points[i - 1];
                for (int dash = 0; dash < 16; dash += 2) {
                    auto interpolate = [&](double t) {
                        return Waypoint{a.x + (p.x - a.x) * t, a.y + (p.y - a.y) * t, 0};
                    };
                    line(interpolate(dash / 16.), interpolate((dash + 1) / 16.), pointColor, 2);
                }
            }
            sphere(p, pointColor, static_cast<int>(i) == overlay.selectedPoint ? .14 : .10,
                   static_cast<int>(i + 1));
            line(p, {p.x + .4 * std::cos(p.yaw), p.y + .4 * std::sin(p.yaw), 0}, pointColor, .025,
                 mjGEOM_ARROW);
        }
        if (overlay.pointPreview)
            sphere(*overlay.pointPreview, {.3f, .7f, 1.f, .45f}, .14);
        if (overlay.boxPreview and scene.value.ngeom < scene.value.maxgeom) {
            auto const &p = *overlay.boxPreview;
            auto const size =
                std::array<mjtNum, 3>{obstacleLength / 2, obstacleWidth / 2, obstacleHeight / 2};
            auto const pos = std::array<mjtNum, 3>{p.x, p.y, obstacleHeight / 2};
            auto const color = Color{1.f, .7f, .2f, .4f};
            mjv_initGeom(&scene.value.geoms[scene.value.ngeom++], mjGEOM_BOX, size.data(), pos.data(),
                         nullptr, color.data());
        }
    }
    Model model;
    Data data;
    SceneResource scene;
    std::unique_ptr<ContextResource> context;
    mjvCamera camera{};
    mjvOption options{};
    SceneOverlay overlay;
    std::function<void(ScenePick const &)> pick;
    QPointF pressed{}, previous{};
    bool top = false, dragged = false;
};
SceneView::SceneView(std::string const &file, std::function<void(ScenePick const &)> pick)
    : QOpenGLWindow{NoPartialUpdate}, render{std::make_unique<RenderState>(file, std::move(pick))} {
    auto format = QSurfaceFormat{};
    format.setRenderableType(QSurfaceFormat::OpenGL);
    format.setVersion(2, 1);
    format.setProfile(QSurfaceFormat::NoProfile);
    format.setDepthBufferSize(24);
    format.setSamples(4);
    setFormat(format);
}
SceneView::~SceneView() {
    if (render->context) {
        makeCurrent();
        render->context.reset();
        doneCurrent();
    }
}
void SceneView::initializeGL() { render->context = std::make_unique<ContextResource>(render->model.get()); }
void SceneView::present(SceneState const &state, SceneOverlay const &overlay) {
    if (state.positions.size() != static_cast<std::size_t>(render->model->nq) or
        state.mocapPositions.size() != static_cast<std::size_t>(3 * render->model->nmocap))
        return;
    std::copy(state.positions.begin(), state.positions.end(), render->data->qpos);
    std::copy(state.mocapPositions.begin(), state.mocapPositions.end(), render->data->mocap_pos);
    render->data->time = state.time;
    render->overlay = overlay;
    mj_forward(render->model.get(), render->data.get());
    requestUpdate();
}
void SceneView::paintGL() {
    auto &r = *render;
    if (not r.context)
        return;
    auto const ratio = devicePixelRatio();
    auto const viewport =
        mjrRect{0, 0, static_cast<int>(width() * ratio), static_cast<int>(height() * ratio)};
    mjv_updateScene(r.model.get(), r.data.get(), &r.options, nullptr, &r.camera, mjCAT_ALL, &r.scene.value);
    r.decorate();
    mjr_render(viewport, &r.scene.value, &r.context->value);
}
void SceneView::topView(bool enabled) {
    render->top = enabled;
    render->camera.lookat[0] = 2;
    render->camera.lookat[1] = 0;
    render->camera.lookat[2] = 0;
    render->camera.azimuth = 90;
    render->camera.elevation = enabled ? -90 : -65;
    render->camera.orthographic = enabled ? 1 : 0;
    render->camera.distance = 10;
    resizeGL(width(), height());
    requestUpdate();
}
void SceneView::resizeGL(int width, int height) {
    if (width > 0 and height > 0)
        render->camera.distance = std::max(10., 12. * height / width);
}
void SceneView::mousePressEvent(QMouseEvent *event) {
    render->pressed = render->previous = event->localPos();
    render->dragged = false;
}
void SceneView::mouseMoveEvent(QMouseEvent *event) {
    auto &r = *render;
    auto const delta = event->localPos() - r.previous;
    r.previous = event->localPos();
    if ((event->localPos() - r.pressed).manhattanLength() > 5)
        r.dragged = true;
    if (event->buttons() == Qt::NoButton or height() <= 0)
        return;
    if (r.top and event->buttons().testFlag(Qt::LeftButton))
        return;
    auto const action = event->buttons().testFlag(Qt::RightButton) ? mjMOUSE_MOVE_H : mjMOUSE_ROTATE_H;
    mjv_moveCamera(r.model.get(), action, delta.x() / height(), delta.y() / height(), &r.scene.value,
                   &r.camera);
    requestUpdate();
}
void SceneView::mouseReleaseEvent(QMouseEvent *event) {
    auto &r = *render;
    if (event->button() != Qt::LeftButton or r.dragged or width() <= 0 or height() <= 0)
        return;
    auto point = std::array<mjtNum, 3>{};
    int geom = -1, flex = -1, skin = -1;
    mjv_updateScene(r.model.get(), r.data.get(), &r.options, nullptr, &r.camera, mjCAT_ALL, &r.scene.value);
    auto const body =
        mjv_select(r.model.get(), r.data.get(), &r.options, static_cast<double>(width()) / height(),
                   event->localPos().x() / width(), 1 - event->localPos().y() / height(), &r.scene.value,
                   point.data(), &geom, &flex, &skin);
    if (body < 0)
        return;
    auto result = ScenePick{point[0], point[1], {}, {}};
    if (geom >= 0) {
        auto const slot = r.model->body_mocapid[r.model->geom_bodyid[geom]];
        if (slot >= 0)
            result.obstacle = static_cast<std::size_t>(slot);
    }
    for (std::size_t i = 0; i < r.overlay.points.size(); ++i)
        if (std::hypot(point[0] - r.overlay.points[i].x, point[1] - r.overlay.points[i].y) < .25)
            result.waypoint = i;
    r.pick(result);
}
void SceneView::wheelEvent(QWheelEvent *event) {
    mjv_moveCamera(render->model.get(), mjMOUSE_ZOOM, 0, -.05 * event->angleDelta().y() / 120.,
                   &render->scene.value, &render->camera);
    requestUpdate();
}
} // namespace simulation
