#include <atomic>
#include <chrono>
#include <example_interfaces/action/fibonacci.hpp>
#include <iostream>
#include <lrclexec/ActionServer.h>
#include <lrclexec/ExecuteAction.h>
#include <lrclexec/SignalStop.h>
#include <lrclexec/SpinWithScope.h>
#include <memory>
#include <rclcpp/rclcpp.hpp>
#include <rclcpp_action/create_client.hpp>
#include <string_view>

using namespace std::chrono_literals;
using Action = example_interfaces::action::Fibonacci;

struct CleanupResource {
    explicit CleanupResource(std::atomic<int> &live_) : live{live_} { live.fetch_add(1); }
    ~CleanupResource() { live.fetch_sub(1); }
    std::atomic<int> &live;
};

int main(int argc, char **argv) {
    auto stop = lexec::inplace_stop_source{};
    auto signals = lrclexec::SignalStop{stop};
    auto const normal = argc > 1 and std::string_view{argv[1]} == "--normal";
    auto options = rclcpp::InitOptions{};
    options.shutdown_on_signal = false;
    rclcpp::init(argc, argv, options, rclcpp::SignalHandlerOptions::None);
    try {
        auto node = std::make_shared<rclcpp::Node>("lrclexec_shutdown_fixture");
        auto scheduler = lrclexec::TimerScheduler{node};
        auto scope = lexec::counting_scope{};
        auto live = std::atomic<int>{0};
        auto cleaned = false;
        auto canceled = false;
        auto failed = false;
        auto server =
            lrclexec::make_action_server_preempt<Action>(scheduler, scope, "shutdown_action", [&](auto) {
                auto resource = std::make_shared<CleanupResource>(live);
                auto result = std::make_shared<Action::Result>();
                std::cout << "READY\n" << std::flush;
                return lrclexec::schedule_after(scheduler, normal ? 20ms : 10s) |
                       lexec::then([resource, result]() noexcept { return result; }) |
                       lexec::let_stopped([&, resource] {
                           auto cleanup = lrclexec::schedule_after(scheduler, 100ms) |
                                          lexec::let_value([&, resource]() noexcept {
                                              cleaned =
                                                  node->get_node_base_interface()->get_context()->is_valid();
                                              return lexec::just_stopped();
                                          });
                           return lexec::write_env(
                               std::move(cleanup),
                               lexec::prop{lexec::get_stop_token, lexec::never_stop_token{}});
                       });
            });
        auto client = rclcpp_action::create_client<Action>(node, "shutdown_action");
        if (not client->wait_for_action_server(3s))
            throw std::runtime_error{"discovery failed"};
        auto work = lrclexec::execute_action(scheduler, client, Action::Goal{}) |
                    lexec::then([&](auto) noexcept { stop.request_stop(); }) |
                    lexec::upon_stopped([&]() noexcept {
                        canceled = true;
                        stop.request_stop();
                    }) |
                    lexec::upon_error([&](auto const &) noexcept {
                        failed = true;
                        stop.request_stop();
                    });
        lexec::spawn(std::move(work), scope.get_token());
        auto executor = rclcpp::executors::SingleThreadedExecutor{};
        executor.add_node(node);
        lrclexec::spin_with_scope(executor, scope, stop.get_token());
        if (live.load() != 0 or failed or signals.error() or (not normal and (not cleaned or not canceled)))
            throw std::runtime_error{"exit did not drain the accepted action and its resources"};
        if (not node->get_node_base_interface()->get_context()->is_valid())
            throw std::runtime_error{"ROS context closed before join"};
        std::cout << "DRAINED\n" << std::flush;
        rclcpp::shutdown();
        return 0;
    } catch (std::exception const &error) {
        std::cerr << error.what() << '\n';
        rclcpp::shutdown();
        return 1;
    }
}
