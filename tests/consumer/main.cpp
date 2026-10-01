#include <lrclexec/ActionServer.h>
#include <lrclexec/ExecuteAction.h>
#include <lrclexec/Service.h>
#include <lrclexec/SignalStop.h>
#include <lrclexec/SpinWithScope.h>
#include <rclcpp/rclcpp.hpp>

int main(int argc, char **argv) {
    auto stop = lexec::inplace_stop_source{};
    auto signals = lrclexec::SignalStop{stop};
    auto options = rclcpp::InitOptions{};
    options.shutdown_on_signal = false;
    rclcpp::init(argc, argv, options, rclcpp::SignalHandlerOptions::None);
    auto node = std::make_shared<rclcpp::Node>("installed_lrclexec_consumer");
    auto scheduler = lrclexec::TimerScheduler{node};
    auto scope = lexec::counting_scope{};
    auto completed = false;
    auto work = lexec::schedule(scheduler) | lexec::then([&]() noexcept {
                    completed = true;
                    stop.request_stop();
                }) |
                lexec::upon_error([&](std::exception_ptr) noexcept { stop.request_stop(); });
    lexec::spawn(std::move(work), scope.get_token());
    auto executor = rclcpp::executors::SingleThreadedExecutor{};
    executor.add_node(node);
    lrclexec::spin_with_scope(executor, scope, stop.get_token());
    rclcpp::shutdown();
    return not completed or signals.error() ? 1 : 0;
}
