#pragma once

#include <functional>
#include <lexec/stop_token.hpp>
#include <memory>
#include <system_error>

namespace lrclexec {
namespace detail {
struct SignalStopState;
}

// POSIX application-entry resource: construct before ROS or any worker thread.
// Keep it alive through scope.join(), shutdown, and ROS entity destruction.
// One instance per process; destroy on the constructing thread. The stop source
// must outlive this resource. Existing worker threads cannot be retroactively masked.
struct SignalStop {
    explicit SignalStop(lexec::inplace_stop_source &stop)
        : SignalStop{[&stop]() noexcept { stop.request_stop(); }} {}
    ~SignalStop();
    SignalStop(SignalStop const &) = delete;
    SignalStop &operator=(SignalStop const &) = delete;
    SignalStop(SignalStop &&) = delete;
    SignalStop &operator=(SignalStop &&) = delete;
    std::error_code error() const noexcept;

  private:
    explicit SignalStop(std::function<void()> requestStop);
    std::unique_ptr<detail::SignalStopState> state;
};

} // namespace lrclexec
