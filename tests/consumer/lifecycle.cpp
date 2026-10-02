#include <lrclexec/SpinWithScope.h>
#include <lrclexec/TimerScheduler.h>
#include <rclcpp/rclcpp.hpp>
#include <rclcpp_lifecycle/lifecycle_node.hpp>

int main(int argc, char **argv) {
    rclcpp::init(argc, argv, rclcpp::InitOptions{}, rclcpp::SignalHandlerOptions::None);
    auto node = std::make_shared<rclcpp_lifecycle::LifecycleNode>("installed_lifecycle_consumer");
    auto scheduler = lrclexec::TimerScheduler{node};
    auto scope = lexec::counting_scope{};
    auto stop = lexec::inplace_stop_source{};
    auto completed = false;
    auto work = lrclexec::schedule_after(scheduler, std::chrono::milliseconds{2}) |
                lexec::then([&]() noexcept {
                    completed = true;
                    stop.request_stop();
                }) |
                lexec::upon_error([&](std::exception_ptr) noexcept { stop.request_stop(); });
    lexec::spawn(std::move(work), scope.get_token());
    auto executor = rclcpp::executors::SingleThreadedExecutor{};
    executor.add_node(scheduler.nodeInterfaces().get_node_base_interface());
    node.reset(); // The installed scheduler retains the complete node, including lifecycle resources.
    lrclexec::spin_with_scope(executor, scope, stop.get_token());
    rclcpp::shutdown();
    return completed ? 0 : 1;
}
