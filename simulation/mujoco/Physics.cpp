#include "Physics.h"
#include <algorithm>
#include <array>
#include <cmath>
#include <cstdint>
#include <limits>
#include <mujoco/mujoco.h>
#include <stdexcept>

#if mjVERSION_HEADER != 3008000
#error "This simulation bridge requires the MuJoCo 3.8.0 SDK"
#endif

namespace simulation {
namespace {
constexpr double pi = 3.14159265358979323846;
struct ModelDeleter {
    void operator()(mjModel *model) const noexcept { mj_deleteModel(model); }
};
struct DataDeleter {
    void operator()(mjData *data) const noexcept { mj_deleteData(data); }
};
using Model = std::unique_ptr<mjModel, ModelDeleter>;
using Data = std::unique_ptr<mjData, DataDeleter>;

Model load(std::string const &file) {
    if (mj_version() != mjVERSION_HEADER)
        throw std::runtime_error{"MuJoCo library/header version mismatch"};
    auto error = std::array<char, 1024>{};
    auto model = Model{mj_loadXML(file.c_str(), nullptr, error.data(), static_cast<int>(error.size()))};
    if (not model)
        throw std::runtime_error{error.data()};
    return model;
}

struct PhysicsImpl final : Physics {
    explicit PhysicsImpl(std::string const &file) : model{load(file)}, data{mj_makeData(model.get())} {
        if (not data)
            throw std::bad_alloc{};
        base = id(mjOBJ_BODY, "base_link");
        laser = id(mjOBJ_SITE, "laser");
        left = id(mjOBJ_ACTUATOR, "left_motor");
        right = id(mjOBJ_ACTUATOR, "right_motor");
        auto const wheel = id(mjOBJ_GEOM, "left_tire");
        radius = model->geom_size[3 * wheel];
        track = model->body_pos[3 * id(mjOBJ_BODY, "left_wheel") + 1] -
                model->body_pos[3 * id(mjOBJ_BODY, "right_wheel") + 1];
        if (radius <= 0 or track <= 0 or std::abs(model->opt.timestep - .002) > 1e-9)
            throw std::runtime_error{"invalid differential-drive model geometry/timestep"};
        mj_forward(model.get(), data.get());
    }
    int id(mjtObj type, char const *name) const {
        auto const index = mj_name2id(model.get(), type, name);
        if (index < 0)
            throw std::runtime_error{std::string{"missing model object: "} + name};
        return index;
    }
    void step(Velocity command) override {
        if (not std::isfinite(command.forward) or not std::isfinite(command.yawRate))
            throw std::invalid_argument{"non-finite velocity command"};
        command.forward = std::clamp(command.forward, -.6, .6);
        command.yawRate = std::clamp(command.yawRate, -1.2, 1.2);
        data->ctrl[left] = (command.forward - command.yawRate * track / 2) / radius;
        data->ctrl[right] = (command.forward + command.yawRate * track / 2) / radius;
        mj_step(model.get(), data.get());
        mj_forward(model.get(), data.get());
        for (int i = 0; i < data->ncon; ++i) {
            auto const &contact = data->contact[i];
            for (auto const order : {0, 1}) {
                auto const robot = contact.geom[order];
                auto const world = contact.geom[1 - order];
                if (robot >= 0 and world >= 0 and model->geom_group[robot] == 1 and
                    model->geom_group[world] != 1 and model->geom_type[world] != mjGEOM_PLANE)
                    obstacleContact = true;
            }
        }
        auto const current = state();
        if (not std::isfinite(current.time) or not std::isfinite(current.x) or not std::isfinite(current.y) or
            not std::isfinite(current.z) or not std::isfinite(current.yaw) or
            not std::isfinite(current.velocity.forward) or not std::isfinite(current.velocity.yawRate) or
            data->warning[mjWARN_BADQPOS].number or data->warning[mjWARN_BADQVEL].number or
            data->warning[mjWARN_BADQACC].number)
            throw std::runtime_error{"MuJoCo unstable simulation"};
    }
    State state() const override {
        auto const *position = data->xpos + 3 * base;
        auto const *q = data->xquat + 4 * base;
        auto const yaw = std::atan2(2 * (q[0] * q[3] + q[1] * q[2]), 1 - 2 * (q[2] * q[2] + q[3] * q[3]));
        auto velocity = std::array<mjtNum, 6>{};
        mj_objectVelocity(model.get(), data.get(), mjOBJ_XBODY, base, velocity.data(), 1);
        return {data->time,
                position[0],
                position[1],
                yaw,
                position[2],
                {q[0], q[1], q[2], q[3]},
                {velocity[3], velocity[2]},
                obstacleContact};
    }
    SceneState scene() const override { return {{data->qpos, data->qpos + model->nq}, data->time}; }
    Scan scan() const override {
        auto scan =
            Scan{std::vector<float>(360), static_cast<float>(-pi), static_cast<float>(2 * pi / 360), 8.f};
        auto const *origin = data->site_xpos + 3 * laser;
        auto const *rotation = data->site_xmat + 9 * laser;
        auto const groups = std::array<mjtByte, mjNGROUP>{1, 0, 1, 0, 0, 0};
        for (std::size_t i = 0; i < scan.ranges.size(); ++i) {
            auto const angle = scan.angleMin + static_cast<double>(i) * scan.angleStep;
            auto const x = std::cos(angle), y = std::sin(angle);
            auto const ray =
                std::array<mjtNum, 3>{rotation[0] * x + rotation[1] * y, rotation[3] * x + rotation[4] * y,
                                      rotation[6] * x + rotation[7] * y};
            int geometry = -1;
            auto const distance =
                mj_ray(model.get(), data.get(), origin, ray.data(), groups.data(), 1, -1, &geometry, nullptr);
            scan.ranges[i] = distance < 0 or distance > scan.rangeMax ? std::numeric_limits<float>::infinity()
                                                                      : static_cast<float>(distance);
        }
        return scan;
    }
    Map map(bool const includeObstacle) const override {
        auto map = Map{std::vector<std::int8_t>(180 * 140, 0), 180, 140, .05, -2.5, -3.5};
        for (unsigned row = 0; row < map.height; ++row)
            for (unsigned col = 0; col < map.width; ++col) {
                auto const x = map.originX + (col + .5) * map.resolution;
                auto const y = map.originY + (row + .5) * map.resolution;
                for (int geom = 0; geom < model->ngeom; ++geom) {
                    if (model->geom_type[geom] != mjGEOM_BOX or model->geom_group[geom] == 1 or
                        (not includeObstacle and model->geom_group[geom] == 2))
                        continue;
                    auto const *position = data->geom_xpos + 3 * geom;
                    auto const *size = model->geom_size + 3 * geom;
                    if (std::abs(x - position[0]) <= size[0] + map.resolution / 2 and
                        std::abs(y - position[1]) <= size[1] + map.resolution / 2)
                        map.cells[row * map.width + col] = 100;
                }
            }
        return map;
    }

    Model model;
    Data data;
    int base = -1, laser = -1, left = -1, right = -1;
    double radius = 0, track = 0;
    bool obstacleContact = false;
};
} // namespace

std::unique_ptr<Physics> makePhysics(std::string const &modelFile) {
    return std::make_unique<PhysicsImpl>(modelFile);
}
} // namespace simulation
