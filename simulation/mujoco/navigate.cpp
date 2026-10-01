#include "Navigator.h"
#include <atomic>
#include <chrono>
#include <iostream>
#include <lexec/coro/co2.hpp>
#include <lifecycle_msgs/msg/state.hpp>
#include <lifecycle_msgs/srv/get_state.hpp>
#include <lrclexec/Service.h>
#include <lrclexec/SignalStop.h>
#include <lrclexec/SpinWithScope.h>
#include <rclcpp/rclcpp.hpp>
#include <rclcpp_action/create_client.hpp>

using namespace std::chrono_literals;
namespace {
struct StartupStopped {};
struct ActiveState {
    std::atomic<bool> ready{false};
    bool active = false;
    bool stopped = false;
    std::exception_ptr error;
};
struct ActiveReceiver {
    using receiver_concept = lexec::receiver_t;
    void set_value(bool active) && noexcept {
        state->active = active;
        state->ready = true;
    }
    void set_stopped() && noexcept {
        state->stopped = true;
        state->ready = true;
    }
    void set_error(std::exception_ptr error) && noexcept {
        state->error = error;
        state->ready = true;
    }
    ActiveState *state;
};
bool waitActive(lrclexec::TimerScheduler const &scheduler, rclcpp::Executor &executor,
                lexec::inplace_stop_token stop, char const *name) {
    using Service = lifecycle_msgs::srv::GetState;
    auto client = scheduler.node().create_client<Service>(std::string{name} + "/get_state");
    auto const deadline = std::chrono::steady_clock::now() + 25s;
    while (not stop.stop_requested() and std::chrono::steady_clock::now() < deadline) {
        auto query =
            lrclexec::call_service(scheduler, client, Service::Request{}) |
            lexec::then([](Service::Response::SharedPtr response) noexcept {
                return response->current_state.id == lifecycle_msgs::msg::State::PRIMARY_STATE_ACTIVE;
            });
        auto timeout =
            lrclexec::schedule_after(scheduler, 500ms) | lexec::then([]() noexcept { return false; });
        auto state = ActiveState{};
        auto localStop = lexec::inplace_stop_source{};
        auto forwardStop =
            lexec::inplace_stop_callback{stop, [&localStop]() noexcept { localStop.request_stop(); }};
        auto operation =
            lexec::connect(lexec::write_env(lexec::when_any(std::move(query), std::move(timeout)),
                                            lexec::prop{lexec::get_stop_token, localStop.get_token()}),
                           ActiveReceiver{&state});
        lexec::start(operation);
        std::exception_ptr spinError;
        while (not state.ready.load()) {
            try {
                executor.spin_once(50ms);
            } catch (...) {
                if (not spinError)
                    spinError = std::current_exception();
                localStop.request_stop();
            }
        }
        if (spinError)
            std::rethrow_exception(spinError);
        if (state.error)
            std::rethrow_exception(state.error);
        if (state.stopped)
            throw StartupStopped{};
        if (state.active)
            return true;
        executor.spin_once(100ms);
    }
    if (stop.stop_requested())
        throw StartupStopped{};
    return false;
}
} // namespace

int main(int argc, char **argv) {
    auto stop = lexec::inplace_stop_source{};
    auto signals = lrclexec::SignalStop{stop};
    auto options = rclcpp::InitOptions{};
    options.shutdown_on_signal = false;
    rclcpp::init(argc, argv, options, rclcpp::SignalHandlerOptions::None);
    auto status = 0;
    try {
        auto nodeOptions =
            rclcpp::NodeOptions{}.parameter_overrides({rclcpp::Parameter{"use_sim_time", true}});
        auto node = std::make_shared<rclcpp::Node>("lexec_mujoco_navigator", nodeOptions);
        auto executor = rclcpp::executors::SingleThreadedExecutor{};
        executor.add_node(node);
        std::cout << "WAITING_FOR_NAV2\n" << std::flush;
        auto startupScheduler = lrclexec::TimerScheduler{node};
        if (not waitActive(startupScheduler, executor, stop.get_token(), "planner_server") or
            not waitActive(startupScheduler, executor, stop.get_token(), "controller_server"))
            throw std::runtime_error{"Nav2 lifecycle did not become ACTIVE"};
        auto scheduler = lrclexec::TimerScheduler{node, lrclexec::TimerClock::node};
        auto resources = navigation::Resources{
            scheduler, rclcpp_action::create_client<navigation::Plan>(node, "compute_path_to_pose"),
            rclcpp_action::create_client<navigation::Follow>(node, "follow_path")};
        if (not resources.planner->wait_for_action_server(3s) or
            not resources.controller->wait_for_action_server(3s))
            throw std::runtime_error{"Nav2 action discovery failed"};
        resources.plannerId = "GridBased";
        resources.controllerId = "FollowPath";
        resources.goalCheckerId = "goal_checker";
        resources.progressCheckerId = "progress_checker";
        resources.replanInterval = 1s;
        auto planCount = 0;
        auto progressCount = 0;
        resources.pathReady = [&](auto const &) { ++planCount; };
        resources.progress = [&](auto const &) { ++progressCount; };
        auto target = geometry_msgs::msg::PoseStamped{};
        target.header.frame_id = "map";
        target.header.stamp = node->now();
        target.pose.position.x = node->declare_parameter<double>("target_x", 4.0);
        target.pose.position.y = node->declare_parameter<double>("target_y", 0.0);
        target.pose.orientation.w = 1;
        auto scope = lexec::counting_scope{};
        auto work = lexec::coro::as_sender(navigation::navigate(resources, target)) |
                    lexec::then([&]() noexcept {
                        std::cout << "SUCCEEDED\n";
                        stop.request_stop();
                    }) |
                    lexec::upon_stopped([&]() noexcept {
                        std::cout << "STOPPED\n";
                        stop.request_stop();
                    }) |
                    lexec::upon_error([&](std::exception_ptr error) noexcept {
                        status = 1;
                        try {
                            std::rethrow_exception(error);
                        } catch (std::exception const &exception) {
                            std::cerr << exception.what() << '\n';
                        }
                        stop.request_stop();
                    });
        std::cout << "GOAL_STARTED\n" << std::flush;
        lexec::spawn(std::move(work), scope.get_token());
        lrclexec::spin_with_scope(executor, scope, stop.get_token());
        std::cout << "DRAINED plans=" << planCount << " feedback=" << progressCount << '\n' << std::flush;
        if (signals.error())
            status = 1;
    } catch (StartupStopped const &) {
        std::cout << "STOPPED\nDRAINED plans=0\n" << std::flush;
    } catch (std::exception const &error) {
        std::cerr << error.what() << '\n';
        status = 1;
    }
    rclcpp::shutdown();
    return status;
}
