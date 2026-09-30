#include "Navigator.h"
#include <lexec/coro/co2.hpp>
#include <variant>

namespace navigation {
namespace {

template <class Action> struct MapActionFailure {
    Phase phase;
    typename Action::Result::SharedPtr operator()(lrclexec::ActionError<Action> const &error) const {
        auto failure = Failure{phase, error.kind, {}, "Nav2 Action failed"};
        if (error.result) {
            failure.code = error.result->error_code;
            failure.message = error.result->error_msg;
        }
        throw NavigationError{std::move(failure)};
    }
    [[noreturn]] typename Action::Result::SharedPtr operator()(std::exception_ptr error) const {
        std::rethrow_exception(error);
    }
};

struct CheckResult {
    Phase phase;
    template <class Result> std::shared_ptr<Result> operator()(std::shared_ptr<Result> result) const {
        if (not result)
            throw NavigationError{Failure{phase, {}, {}, "Nav2 returned no result"}};
        if (result->error_code != Result::NONE)
            throw NavigationError{Failure{phase, {}, result->error_code, result->error_msg}};
        return result;
    }
};
struct ReachedGoal {};
struct NewPlan {
    Plan::Result::SharedPtr path;
};
using Step = std::variant<ReachedGoal, NewPlan>;

auto plan(Resources const &resources, geometry_msgs::msg::PoseStamped const &target) {
    auto goal = Plan::Goal{};
    goal.goal = target;
    goal.planner_id = resources.plannerId;
    goal.use_start = false;
    return lrclexec::execute_action(resources.scheduler, resources.planner, std::move(goal)) |
           lexec::upon_error(MapActionFailure<Plan>{Phase::planning}) |
           lexec::then(CheckResult{Phase::planning}) |
           lexec::then([observer = resources.pathReady](Plan::Result::SharedPtr result) {
               if (result->path.poses.empty())
                   throw NavigationError{Failure{Phase::planning, {}, {}, "Nav2 returned an empty path"}};
               if (observer)
                   observer(result);
               return result;
           });
}

auto follow(Resources const &resources, Plan::Result::SharedPtr const &path) {
    auto goal = Follow::Goal{};
    goal.path = path->path;
    goal.controller_id = resources.controllerId;
    goal.goal_checker_id = resources.goalCheckerId;
    goal.progress_checker_id = resources.progressCheckerId;
    return lrclexec::execute_action(resources.scheduler, resources.controller, std::move(goal)) |
           lexec::upon_error(MapActionFailure<Follow>{Phase::following}) |
           lexec::then(CheckResult{Phase::following}) |
           lexec::then([](Follow::Result::SharedPtr) noexcept -> Step { return ReachedGoal{}; });
}

auto followOrReplan(Resources const &resources, geometry_msgs::msg::PoseStamped const &target,
                    Plan::Result::SharedPtr const &path) {
    auto replan = lrclexec::schedule_after(resources.scheduler, resources.replanInterval) |
                  lexec::let_value([resources, target] { return plan(resources, target); }) |
                  lexec::then([](Plan::Result::SharedPtr result) noexcept -> Step {
                      return NewPlan{std::move(result)};
                  });
    return lexec::when_any(follow(resources, path), std::move(replan));
}

} // namespace

auto navigate(Resources resources, geometry_msgs::msg::PoseStamped target)
    CO2_BEG(co2::Task<>, (resources, target), Plan::Result::SharedPtr path; Step step;) {
    CO2_AWAIT_SET(path, plan(resources, target));
    while (true) {
        CO2_AWAIT_SET(step, followOrReplan(resources, target, path));
        if (std::holds_alternative<ReachedGoal>(step))
            CO2_RETURN();
        path = std::get<NewPlan>(std::move(step)).path;
    }
}
CO2_END

} // namespace navigation
