#include "GoalInbox.h"
#include <cassert>
#include <cmath>
#include <iomanip>
#include <iostream>
#include <sstream>

namespace simulation {
namespace {
bool normalize(geometry_msgs::msg::PoseStamped &pose) {
    auto &p = pose.pose.position;
    auto &q = pose.pose.orientation;
    auto const norm = std::hypot(q.z, q.w);
    if (pose.header.frame_id != "map" or not std::isfinite(p.x) or not std::isfinite(p.y) or
        not std::isfinite(p.z) or not std::isfinite(q.x) or not std::isfinite(q.y) or
        not std::isfinite(norm) or norm < 1e-9 or std::abs(p.z) > 1e-6 or std::abs(q.x) > 1e-6 or
        std::abs(q.y) > 1e-6)
        return false;
    p.z = 0;
    q.x = q.y = 0;
    q.z /= norm;
    q.w /= norm;
    return true;
}
std::string clean(std::string message) {
    for (auto &c : message)
        if (static_cast<unsigned char>(c) < 32)
            c = ' ';
    return message;
}
} // namespace
void GoalInbox::ready() {
    auto lock = std::lock_guard{mutex};
    if (not state.closed)
        state.ready = true;
    changed.notify_one();
}
bool GoalInbox::submit(geometry_msgs::msg::PoseStamped pose) {
    auto const valid = normalize(pose);
    std::shared_ptr<lexec::inplace_stop_source> previous;
    {
        auto lock = std::lock_guard{mutex};
        if (not valid or not state.ready or state.closed) {
            ++state.rejected;
            state.message = valid ? "Session is not ready" : "Expected a finite planar pose in map";
            return false;
        }
        if (pending)
            ++state.superseded;
        pending =
            SessionGoal{++state.accepted, std::move(pose), std::make_shared<lexec::inplace_stop_source>()};
        state.pending = pending->id;
        state.message.clear();
        previous = active;
        std::cout << "GOAL_ACCEPTED id=" << pending->id << '\n' << std::flush;
    }
    if (previous)
        previous->request_stop();
    changed.notify_one();
    return true;
}
bool GoalInbox::cancel() {
    std::shared_ptr<lexec::inplace_stop_source> previous;
    bool hadWork;
    {
        auto lock = std::lock_guard{mutex};
        hadWork = active or pending;
        if (pending)
            ++state.superseded;
        pending.reset();
        state.pending = 0;
        previous = active;
    }
    if (previous)
        previous->request_stop();
    return hadWork;
}
void GoalInbox::close() {
    std::shared_ptr<lexec::inplace_stop_source> previous;
    {
        auto lock = std::lock_guard{mutex};
        state.closed = true;
        if (pending)
            ++state.superseded;
        pending.reset();
        state.pending = 0;
        previous = active;
    }
    if (previous)
        previous->request_stop();
    changed.notify_one();
}
std::optional<SessionGoal> GoalInbox::next() {
    auto lock = std::unique_lock{mutex};
    assert(not active);
    changed.wait(lock, [&] { return state.closed or (state.ready and pending); });
    if (state.closed)
        return {};
    auto result = std::move(pending);
    pending.reset();
    active = result->stop;
    state.active = result->id;
    state.pending = 0;
    state.message.clear();
    ++state.started;
    std::cout << "GOAL_STARTED id=" << result->id << " x=" << result->pose.pose.position.x
              << " y=" << result->pose.pose.position.y << '\n'
              << std::flush;
    return result;
}
void GoalInbox::complete(std::uint64_t id, GoalOutcome outcome, std::string message) {
    auto lock = std::lock_guard{mutex};
    assert(active and state.active == id);
    if (outcome == GoalOutcome::succeeded) {
        ++state.succeeded;
        state.lastResult = "succeeded";
    } else if (outcome == GoalOutcome::canceled) {
        ++state.canceled;
        state.lastResult = "canceled";
    } else {
        ++state.failed;
        state.lastResult = "failed";
    }
    state.lastId = id;
    state.message = clean(std::move(message));
    state.active = 0;
    active.reset();
    std::cout << "GOAL_DRAINED id=" << id << " result=" << state.lastResult << '\n' << std::flush;
}
SessionStatus GoalInbox::status() const {
    auto lock = std::lock_guard{mutex};
    auto result = state;
    result.stopping = active and active->stop_requested();
    return result;
}
std::string statusJson(SessionStatus const &s) {
    auto const phase = s.active   ? (s.stopping ? "stopping" : "running")
                       : s.closed ? "closed"
                       : s.ready  ? "ready"
                                  : "starting";
    auto text = std::ostringstream{};
    text << "{\"state\":" << std::quoted(phase) << ",\"active\":" << s.active << ",\"pending\":" << s.pending
         << ",\"accepted\":" << s.accepted << ",\"started\":" << s.started << ",\"succeeded\":" << s.succeeded
         << ",\"canceled\":" << s.canceled << ",\"failed\":" << s.failed << ",\"superseded\":" << s.superseded
         << ",\"rejected\":" << s.rejected << ",\"last_id\":" << s.lastId
         << ",\"last_result\":" << std::quoted(s.lastResult) << ",\"message\":" << std::quoted(s.message)
         << "}";
    return text.str();
}
} // namespace simulation
