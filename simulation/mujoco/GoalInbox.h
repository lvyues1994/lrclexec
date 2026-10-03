#pragma once

#include <condition_variable>
#include <cstdint>
#include <geometry_msgs/msg/pose_array.hpp>
#include <geometry_msgs/msg/pose_stamped.hpp>
#include <lexec/execution.hpp>
#include <memory>
#include <mutex>
#include <optional>
#include <string>

namespace simulation {
struct SessionRoute {
    std::uint64_t id;
    std::vector<geometry_msgs::msg::PoseStamped> poses;
    std::shared_ptr<lexec::inplace_stop_source> stop;
};
enum class GoalOutcome { succeeded, canceled, failed };
struct SessionStatus {
    bool ready = false, closed = false, stopping = false;
    std::uint64_t active = 0, pending = 0, accepted = 0, started = 0;
    std::uint64_t succeeded = 0, canceled = 0, failed = 0, superseded = 0, rejected = 0;
    std::uint64_t lastId = 0;
    std::size_t waypointIndex = 0, waypointCount = 0, waypointCompleted = 0;
    std::string lastResult, message;
};

// A bounded mailbox with one consumer. next() installs the active stop source
// atomically with taking the latest target. The consumer must destroy the old
// operation before complete(), and call complete() before next().
class GoalInbox {
  public:
    void ready();
    bool submit(geometry_msgs::msg::PoseStamped pose);
    bool submit(geometry_msgs::msg::PoseArray route);
    bool cancel();
    void close();
    std::optional<SessionRoute> next();
    bool beginWaypoint(std::uint64_t id, std::size_t index);
    void reachedWaypoint(std::uint64_t id);
    void complete(std::uint64_t id, GoalOutcome outcome, std::string message = {});
    SessionStatus status() const;

  private:
    mutable std::mutex mutex;
    std::condition_variable changed;
    std::optional<SessionRoute> pending;
    std::shared_ptr<lexec::inplace_stop_source> active;
    SessionStatus state;
};
std::string statusJson(SessionStatus const &status);
} // namespace simulation
