#include "Viewer.h"
#include <stdexcept>

#ifdef LRCLEXEC_MUJOCO_VIEWER
#define GLFW_INCLUDE_NONE
#include <GLFW/glfw3.h>
#include <algorithm>
#include <array>
#include <cmath>
#include <cstdio>
#include <filesystem>
#include <fstream>
#include <mujoco/mujoco.h>
#include <sstream>

namespace simulation {
namespace {
struct ModelDeleter {
    void operator()(mjModel *model) const noexcept { mj_deleteModel(model); }
};
struct DataDeleter {
    void operator()(mjData *data) const noexcept { mj_deleteData(data); }
};
struct WindowDeleter {
    void operator()(GLFWwindow *window) const noexcept { glfwDestroyWindow(window); }
};
using Model = std::unique_ptr<mjModel, ModelDeleter>;
using Data = std::unique_ptr<mjData, DataDeleter>;
using Window = std::unique_ptr<GLFWwindow, WindowDeleter>;

Model load(std::string const &file) {
    auto error = std::array<char, 1024>{};
    auto model = Model{mj_loadXML(file.c_str(), nullptr, error.data(), static_cast<int>(error.size()))};
    if (not model)
        throw std::runtime_error{error.data()};
    return model;
}
struct Glfw {
    Glfw() {
        glfwSetErrorCallback(
            [](int code, char const *text) { std::fprintf(stderr, "GLFW %d: %s\n", code, text); });
        if (not glfwInit())
            throw std::runtime_error{"GLFW initialization failed; check DISPLAY"};
    }
    ~Glfw() { glfwTerminate(); }
    Glfw(Glfw const &) = delete;
    Glfw &operator=(Glfw const &) = delete;
};
Window createWindow() {
    auto window = Window{glfwCreateWindow(1200, 900, "MuJoCo + Nav2", nullptr, nullptr)};
    if (not window)
        throw std::runtime_error{"cannot create MuJoCo OpenGL window"};
    glfwMakeContextCurrent(window.get());
    // ROS physics uses a wall timer; v-sync must not stall that executor.
    glfwSwapInterval(0);
    return window;
}
struct Scene {
    mjvScene value{};
    explicit Scene(mjModel const *model) {
        mjv_defaultScene(&value);
        mjv_makeScene(model, &value, 4096);
    }
    ~Scene() { mjv_freeScene(&value); }
    Scene(Scene const &) = delete;
    Scene &operator=(Scene const &) = delete;
};
struct Context {
    mjrContext value{};
    explicit Context(mjModel const *model) {
        mjr_defaultContext(&value);
        mjr_makeContext(model, &value, mjFONTSCALE_150);
    }
    ~Context() { mjr_freeContext(&value); }
    Context(Context const &) = delete;
    Context &operator=(Context const &) = delete;
};

struct ViewerImpl final : Viewer {
    explicit ViewerImpl(ViewerConfig const &config_)
        : config{config_}, model{load(config.modelFile)}, data{mj_makeData(model.get())},
          window{createWindow()}, scene{model.get()}, context{model.get()} {
        if (not data)
            throw std::bad_alloc{};
        mjv_defaultCamera(&camera);
        mjv_defaultOption(&options);
        options.geomgroup[3] = 1;
        camera.lookat[0] = 2;
        camera.distance = 9;
        camera.azimuth = 90;
        camera.elevation = -70;
        glfwSetWindowUserPointer(window.get(), this);
        glfwSetKeyCallback(window.get(), [](GLFWwindow *handle, int key, int, int action, int) {
            if (key == GLFW_KEY_ESCAPE and action == GLFW_PRESS)
                glfwSetWindowShouldClose(handle, GLFW_TRUE);
        });
        glfwSetScrollCallback(window.get(), [](GLFWwindow *handle, double, double offset) {
            auto &self = *static_cast<ViewerImpl *>(glfwGetWindowUserPointer(handle));
            mjv_moveCamera(self.model.get(), mjMOUSE_ZOOM, 0, -.05 * offset, &self.scene.value, &self.camera);
        });
        glfwSetCursorPosCallback(window.get(), [](GLFWwindow *handle, double x, double y) {
            auto &self = *static_cast<ViewerImpl *>(glfwGetWindowUserPointer(handle));
            self.drag(x, y);
        });
    }
    void setGoal(double x, double y) override {
        config.targetX = x;
        config.targetY = y;
        config.hasTarget = true;
    }
    void drag(double x, double y) {
        auto const dx = x - cursorX, dy = y - cursorY;
        cursorX = x;
        cursorY = y;
        auto const left = glfwGetMouseButton(window.get(), GLFW_MOUSE_BUTTON_LEFT) == GLFW_PRESS;
        auto const right = glfwGetMouseButton(window.get(), GLFW_MOUSE_BUTTON_RIGHT) == GLFW_PRESS;
        if (not left and not right)
            return;
        int width = 0, height = 0;
        glfwGetWindowSize(window.get(), &width, &height);
        if (height > 0)
            mjv_moveCamera(model.get(), right ? mjMOUSE_MOVE_H : mjMOUSE_ROTATE_H, dx / height, dy / height,
                           &scene.value, &camera);
    }
    bool present(SceneState const &state) override {
        glfwPollEvents();
        if (glfwWindowShouldClose(window.get())) {
            glfwHideWindow(window.get());
            return false;
        }
        if (state.positions.size() != static_cast<std::size_t>(model->nq) or
            state.mocapPositions.size() != static_cast<std::size_t>(3 * model->nmocap))
            throw std::invalid_argument{"viewer/physics model state mismatch"};
        std::copy(state.positions.begin(), state.positions.end(), data->qpos);
        std::copy(state.mocapPositions.begin(), state.mocapPositions.end(), data->mocap_pos);
        data->time = state.time;
        mj_forward(model.get(), data.get());
        auto viewport = mjrRect{0, 0, 0, 0};
        glfwGetFramebufferSize(window.get(), &viewport.width, &viewport.height);
        if (viewport.width <= 0 or viewport.height <= 0)
            return true;
        mjv_updateScene(model.get(), data.get(), &options, nullptr, &camera, mjCAT_ALL, &scene.value);
        addGoal();
        mjr_render(viewport, &scene.value, &context.value);
        auto text = std::ostringstream{};
        text << "Nav2 + MuJoCo\nTime: " << state.time << " s\nRobot: (" << state.positions[0] << ", "
             << state.positions[1] << ")";
        if (config.hasTarget)
            text << "\nLast target: (" << config.targetX << ", " << config.targetY << ")";
        else
            text << "\nWaiting for a target in RViz";
        text << "\nDrag: rotate | Right drag: pan | Wheel: zoom\nEsc / Close: stop";
        mjr_overlay(mjFONT_NORMAL, mjGRID_TOPLEFT, viewport, text.str().c_str(), nullptr, &context.value);
        if (not config.captureFile.empty() and state.time - capturedAt >= 1) {
            capture(viewport);
            capturedAt = state.time;
        }
        glfwSwapBuffers(window.get());
        return true;
    }
    void addGoal() {
        if (not config.hasTarget or scene.value.ngeom == scene.value.maxgeom)
            return;
        auto const size = std::array<mjtNum, 3>{.09, .09, .09};
        auto const position = std::array<mjtNum, 3>{config.targetX, config.targetY, .09};
        auto const color = std::array<float, 4>{.1f, .9f, .3f, .8f};
        auto &goal = scene.value.geoms[scene.value.ngeom++];
        mjv_initGeom(&goal, mjGEOM_SPHERE, size.data(), position.data(), nullptr, color.data());
    }
    void capture(mjrRect viewport) {
        auto pixels = std::vector<unsigned char>(static_cast<std::size_t>(viewport.width) *
                                                 static_cast<std::size_t>(viewport.height) * 3);
        mjr_readPixels(pixels.data(), nullptr, viewport, &context.value);
        auto const temporary = config.captureFile + ".tmp";
        auto file = std::ofstream{temporary, std::ios::binary};
        file << "P6\n" << viewport.width << ' ' << viewport.height << "\n255\n";
        auto const rowBytes = static_cast<std::streamsize>(viewport.width) * 3;
        for (int row = viewport.height - 1; row >= 0; --row)
            file.write(reinterpret_cast<char const *>(pixels.data() + row * rowBytes), rowBytes);
        file.close();
        if (not file)
            throw std::runtime_error{"cannot save viewer capture"};
        std::filesystem::rename(temporary, config.captureFile);
    }
    ViewerConfig config;
    Model model;
    Data data;
    Glfw glfw;
    Window window;
    Scene scene;
    Context context;
    mjvCamera camera{};
    mjvOption options{};
    double cursorX = 0, cursorY = 0, capturedAt = -1;
};
} // namespace
std::unique_ptr<Viewer> makeViewer(ViewerConfig const &config) {
    return std::make_unique<ViewerImpl>(config);
}
} // namespace simulation
#else
namespace simulation {
std::unique_ptr<Viewer> makeViewer(ViewerConfig const &) {
    throw std::runtime_error{"rebuild with LRCLEXEC_MUJOCO_VIEWER=ON to open the window"};
}
} // namespace simulation
#endif
