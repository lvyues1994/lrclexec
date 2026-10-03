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
        check(a and a->pose.pose.orientation.w == 1, "goal not normalized");
        check(inbox.submit(target(2)) and inbox.submit(target(3)), "replacement rejected");
        check(a->stop->stop_requested(), "active target not stopped");
        check(inbox.status().started == 1, "replacement started before old operation drained");
        inbox.complete(a->id, simulation::GoalOutcome::canceled);
        auto c = inbox.next();
        check(c and c->id == 3 and c->pose.pose.position.x == 3, "did not retain only newest target");
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
        std::cout << "latest target, cancel/reissue, error recovery and idle/active close passed\n";
    } catch (std::exception const &error) {
        std::cerr << error.what() << '\n';
        return 1;
    }
}
