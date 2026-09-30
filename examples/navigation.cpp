#include <chrono>
#include <example_interfaces/action/fibonacci.hpp>
#include <iostream>
#include <lexec/any_sender_of.hpp>
#include <lrclexec/ActionServer.h>
#include <lrclexec/ExecuteAction.h>
#include <lrclexec/SpinWithScope.h>
#include <rclcpp/rclcpp.hpp>
#include <rclcpp_action/rclcpp_action.hpp>
#include <variant>

using namespace std::chrono_literals;
using Action = example_interfaces::action::Fibonacci;
using Result = Action::Result::SharedPtr;
using Navigation =
    lexec::any_sender_of<lexec::set_value_t(Result), lexec::set_error_t(lrclexec::ActionError<Action>),
                         lexec::set_error_t(std::exception_ptr), lexec::set_stopped_t()>;

struct NavigationResources {
    lrclexec::TimerScheduler scheduler;
    rclcpp_action::Client<Action>::SharedPtr planner;
    rclcpp_action::Client<Action>::SharedPtr controller;
};
struct ReachedGoal {};
struct NewPlan {
    Result path;
};
using Step = std::variant<ReachedGoal, NewPlan>;

auto execute(NavigationResources const &resources, rclcpp_action::Client<Action>::SharedPtr client,
             int const duration) {
    auto goal = Action::Goal{};
    goal.order = duration;
    return lrclexec::execute_action(resources.scheduler, std::move(client), std::move(goal));
}

Navigation follow(NavigationResources const &resources, Result path, int const remaining) {
    std::cout << "follow path " << path->sequence.front() << '\n';
    auto controller = execute(resources, resources.controller, remaining == 0 ? 5 : 1000) |
                      lexec::then([](Result) noexcept -> Step { return ReachedGoal{}; });
    auto replan = lrclexec::schedule_after(resources.scheduler, 30ms) |
                  lexec::let_value([resources] { return execute(resources, resources.planner, 15); }) |
                  lexec::then([](Result next) noexcept -> Step { return NewPlan{std::move(next)}; });
    return Navigation{lexec::when_any(std::move(controller), std::move(replan)) |
                      lexec::let_value([resources, remaining](Step step) -> Navigation {
                          if (std::holds_alternative<ReachedGoal>(step)) {
                              auto result = std::make_shared<Action::Result>();
                              result->sequence = {1};
                              return Navigation{lexec::just(std::move(result))};
                          }
                          return follow(resources, std::get<NewPlan>(std::move(step)).path, remaining - 1);
                      })};
}

int main(int argc, char **argv) {
    rclcpp::init(argc, argv, rclcpp::InitOptions{}, rclcpp::SignalHandlerOptions::None);
    auto node = std::make_shared<rclcpp::Node>("lrclexec_navigation_example");
    auto scheduler = lrclexec::TimerScheduler{node};
    auto scope = lexec::counting_scope{};
    auto stop = lexec::inplace_stop_source{};
    auto version = 0;
    auto failed = false;
    auto planner = lrclexec::make_action_server_preempt<Action>(scheduler, scope, "demo_planner", [&](auto) {
        auto result = std::make_shared<Action::Result>();
        result->sequence = {++version};
        return lrclexec::schedule_after(scheduler, 15ms) |
               lexec::then([result]() noexcept { return result; });
    });
    auto controller = lrclexec::make_action_server_preempt<Action>(
        scheduler, scope, "demo_controller", [scheduler](auto handle) {
            auto result = std::make_shared<Action::Result>();
            return lrclexec::schedule_after(scheduler, std::chrono::milliseconds{handle->get_goal()->order}) |
                   lexec::then([result]() noexcept { return result; });
        });
    auto resources =
        NavigationResources{scheduler, rclcpp_action::create_client<Action>(node, "demo_planner"),
                            rclcpp_action::create_client<Action>(node, "demo_controller")};
    if (not resources.planner->wait_for_action_server(3s) or
        not resources.controller->wait_for_action_server(3s)) {
        std::cerr << "local action discovery failed\n";
        rclcpp::shutdown();
        return 1;
    }
    auto navigation =
        execute(resources, resources.planner, 15) |
        lexec::let_value([resources](Result path) { return follow(resources, std::move(path), 2); }) |
        lexec::then([&](Result) noexcept {
            std::cout << "reached goal\n";
            stop.request_stop();
        }) |
        lexec::upon_error([&](auto const &) noexcept {
            failed = true;
            stop.request_stop();
        });
    lexec::spawn(std::move(navigation), scope.get_token());
    auto executor = rclcpp::executors::SingleThreadedExecutor{};
    executor.add_node(node);
    lrclexec::spin_with_scope(executor, scope, stop.get_token());
    rclcpp::shutdown();
    return failed or version != 3 ? 1 : 0;
}
