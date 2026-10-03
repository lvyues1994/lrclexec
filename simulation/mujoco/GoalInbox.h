#pragma once

#include <condition_variable>
#include <cstdint>
#include <geometry_msgs/msg/pose_stamped.hpp>
#include <lexec/execution.hpp>
#include <memory>
#include <mutex>
#include <optional>
#include <string>

namespace simulation {
struct SessionGoal {
    std::uint64_t id;
    geometry_msgs::msg::PoseStamped pose;
    std::shared_ptr<lexec::inplace_stop_source> stop;
};
enum class GoalOutcome { succeeded, canceled, failed };
struct SessionStatus {
    bool ready = false, closed = false, stopping = false;
    std::uint64_t active = 0, pending = 0, accepted = 0, started = 0;
    std::uint64_t succeeded = 0, canceled = 0, failed = 0, superseded = 0, rejected = 0;
    std::uint64_t lastId = 0;
    std::string lastResult, message;
};

// A bounded mailbox with one consumer. next() installs the active stop source
// atomically with taking the latest target. The consumer must destroy the old
// operation before complete(), and call complete() before next().
class GoalInbox {
  public:
    void ready();
    bool submit(geometry_msgs::msg::PoseStamped pose);
    bool cancel();
    void close();
    std::optional<SessionGoal> next();
    void complete(std::uint64_t id, GoalOutcome outcome, std::string message = {});
    SessionStatus status() const;

  private:
    mutable std::mutex mutex;
    std::condition_variable changed;
    std::optional<SessionGoal> pending;
    std::shared_ptr<lexec::inplace_stop_source> active;
    SessionStatus state;
};
std::string statusJson(SessionStatus const &status);
} // namespace simulation
