#pragma once

#include <exception>
#include <memory>
#include <mutex>
#include <utility>

namespace lrclexec::detail {

// An independently owned receiver slot. Late ROS callbacks never refer to an op.
template <class Receiver> struct Completion {
    explicit Completion(Receiver receiver_) : receiver{std::make_unique<Receiver>(std::move(receiver_))} {}
    Completion(Completion &&) = delete;

    std::unique_ptr<Receiver> takeReceiver() noexcept {
        auto const lock = std::lock_guard<std::mutex>{mutex};
        auto result = std::move(receiver);
        return result;
    }
    bool isComplete() const noexcept {
        auto const lock = std::lock_guard<std::mutex>{mutex};
        return not receiver;
    }
    void rememberError(std::exception_ptr error_) noexcept {
        auto const lock = std::lock_guard<std::mutex>{mutex};
        error = std::move(error_);
    }
    std::exception_ptr pendingError() const noexcept {
        auto const lock = std::lock_guard<std::mutex>{mutex};
        return error;
    }

  private:
    mutable std::mutex mutex;
    std::unique_ptr<Receiver> receiver;
    std::exception_ptr error;
};

} // namespace lrclexec::detail
