#pragma once

#include <atomic>
#include <chrono>
#include <exception>
#include <lexec/execution.hpp>
#include <rclcpp/executor.hpp>
#include <stdexcept>

namespace lrclexec {
namespace detail {
struct JoinReceiver {
    using receiver_concept = lexec::receiver_t;
    void set_value() && noexcept { joined->store(true, std::memory_order_release); }
    std::atomic<bool> *joined;
};
} // namespace detail

// Call outside executor callbacks, with automatic ROS signal shutdown disabled.
// The caller owns shutdown, after this function has drained the scope.
inline void spin_with_scope(rclcpp::Executor &executor, lexec::counting_scope &scope,
                            lexec::inplace_stop_token const stop) {
    std::exception_ptr failure;
    try {
        while (not stop.stop_requested())
            executor.spin_once(std::chrono::milliseconds{10});
    } catch (...) {
        failure = std::current_exception();
    }
    scope.close();
    scope.request_stop();
    auto joined = std::atomic<bool>{false};
    auto join = lexec::connect(scope.join(), detail::JoinReceiver{&joined});
    lexec::start(join);
    while (not joined.load(std::memory_order_acquire)) {
        try {
            executor.spin_once(std::chrono::milliseconds{10});
        } catch (...) {
            if (not failure)
                failure = std::current_exception();
        }
    }
    if (failure)
        std::rethrow_exception(failure);
}

} // namespace lrclexec
