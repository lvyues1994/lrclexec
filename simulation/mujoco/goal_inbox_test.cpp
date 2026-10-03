#include "GoalInbox.h"
#include <chrono>
#include <future>
#include <iostream>
#include <limits>
#include <stdexcept>

using namespace std::chrono_literals;
namespace {
void check(bool condition, char const *message) {
    if (not condition)
        throw std::runtime_error{message};
}
geometry_msgs::msg::PoseStamped target(double x) {
    auto pose = geometry_msgs::msg::PoseStamped{};
    pose.header.frame_id = "map";
    pose.pose.position.x = x;
    pose.pose.orientation.w = 2; // Valid, normalized at the boundary.
    return pose;
}
} // namespace
int main() {
    try {
        auto inbox = simulation::GoalInbox{};
        check(not inbox.submit(target(1)), "accepted before readiness");
        inbox.ready();
        check(not inbox.submit(target(std::numeric_limits<double>::infinity())), "accepted infinity");
        auto invalid = target(1);
        invalid.header.frame_id = "odom";
        check(not inbox.submit(invalid), "accepted wrong frame");
        invalid = target(1);
        invalid.pose.orientation.w = 0;
        check(not inbox.submit(invalid), "accepted zero quaternion");
        check(inbox.submit(target(1)), "first goal rejected");
        auto a = inbox.next();
        check(a and a->poses.front().pose.orientation.w == 1, "goal not normalized");
        check(inbox.submit(target(2)) and inbox.submit(target(3)), "replacement rejected");
        check(a->stop->stop_requested(), "active target not stopped");
        check(inbox.status().started == 1, "replacement started before old operation drained");
        inbox.complete(a->id, simulation::GoalOutcome::canceled);
        auto c = inbox.next();
        check(c and c->id == 3 and c->poses.front().pose.position.x == 3,
              "did not retain only newest target");
        check(inbox.submit(target(4)) and inbox.cancel(), "cancel missed pending goal");
        check(c->stop->stop_requested() and inbox.status().pending == 0, "cancel left queued work");
        inbox.complete(c->id, simulation::GoalOutcome::canceled);
        check(inbox.submit(target(5)), "could not reissue after cancellation");
        auto e = inbox.next();
        inbox.complete(e->id, simulation::GoalOutcome::failed, "planner failed\nretry allowed");
        check(inbox.submit(target(6)), "could not recover after failed navigation");
        auto f = inbox.next();
        inbox.complete(f->id, simulation::GoalOutcome::succeeded);
        check(not inbox.cancel(), "idle cancel reported active work");
        auto waiting = std::async(std::launch::async, [&] { return inbox.next(); });
        check(waiting.wait_for(20ms) == std::future_status::timeout, "idle consumer did not wait");
        inbox.close();
        if (waiting.wait_for(1s) != std::future_status::ready) {
            std::cerr << "idle close hung\n" << std::flush;
            std::_Exit(1); // A failed wake must not hang in future's destructor.
        }
        check(not waiting.get(), "idle close produced work");
        check(not inbox.submit(target(7)), "closed inbox accepted target");
        auto const s = inbox.status();
        check(s.started == 4 and s.superseded == 2 and s.canceled == 2 and s.failed == 1 and s.succeeded == 1,
              "mission accounting mismatch");
        auto closing = simulation::GoalInbox{};
        closing.ready();
        closing.submit(target(1));
        auto active = closing.next();
        closing.submit(target(2));
        closing.close();
        check(active->stop->stop_requested() and closing.status().pending == 0,
              "close did not stop all work");
        closing.complete(active->id, simulation::GoalOutcome::canceled);
        check(not closing.next(), "close ran a queued target");
        auto routes = simulation::GoalInbox{};
        routes.ready();
        auto route = geometry_msgs::msg::PoseArray{};
        route.header.frame_id = "map";
        check(not routes.submit(route), "accepted empty route");
        route.poses = {target(1).pose, target(2).pose, target(3).pose};
        auto invalidRoute = route;
        invalidRoute.poses[1].orientation.w = 0;
        check(not routes.submit(invalidRoute), "partially accepted invalid route");
        invalidRoute.poses.assign(65, target(1).pose);
        check(not routes.submit(invalidRoute), "accepted unbounded route");
        check(routes.submit(route), "route rejected");
        auto task = routes.next();
        check(not routes.submit(invalidRoute) and not task->stop->stop_requested(),
              "invalid route interrupted active task");
        check(task->poses.size() == 3 and routes.beginWaypoint(task->id, 0), "route did not begin");
        routes.reachedWaypoint(task->id);
        check(routes.status().waypointCompleted == 1, "lost waypoint arrival");
        routes.cancel();
        check(not routes.beginWaypoint(task->id, 1), "cancellation started next waypoint");
        routes.complete(task->id, simulation::GoalOutcome::canceled);
        routes.submit(route);
        task = routes.next();
        for (std::size_t index = 0; index < 3; ++index) {
            check(routes.beginWaypoint(task->id, index), "next waypoint not allowed");
            routes.reachedWaypoint(task->id);
        }
        routes.complete(task->id, simulation::GoalOutcome::succeeded);
        check(routes.status().succeeded == 1 and routes.status().waypointCompleted == 3,
              "route/waypoint accounting mixed");
        routes.submit(target(1));
        task = routes.next();
        check(routes.submit(route) and task->stop->stop_requested(), "route did not replace single goal");
        routes.complete(task->id, simulation::GoalOutcome::canceled);
        task = routes.next();
        check(task->poses.size() == 3, "single-to-route replacement lost points");
        check(routes.submit(target(2)) and not routes.beginWaypoint(task->id, 0),
              "single replacement left old route runnable");
        routes.complete(task->id, simulation::GoalOutcome::canceled);
        task = routes.next();
        routes.beginWaypoint(task->id, 0);
        routes.reachedWaypoint(task->id);
        routes.cancel(); // Physical arrival wins over a late cancellation, as in single-goal mode.
        routes.complete(task->id, simulation::GoalOutcome::succeeded);
        check(routes.status().lastResult == "succeeded", "late cancellation changed completed result");
        routes.close();
        std::cout << "latest target, cancel/reissue, error recovery and idle/active close passed\n";
    } catch (std::exception const &error) {
        std::cerr << error.what() << '\n';
        return 1;
    }
}
