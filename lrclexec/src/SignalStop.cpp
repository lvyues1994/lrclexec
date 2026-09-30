#include <atomic>
#include <cerrno>
#include <csignal>
#include <lrclexec/SignalStop.h>
#include <pthread.h>
#include <stdexcept>
#include <thread>

namespace lrclexec::detail {
namespace {

struct SignalOwnership {
    SignalOwnership() {
        if (active().exchange(true))
            throw std::logic_error{"lrclexec: SignalStop already exists"};
    }
    ~SignalOwnership() { active().store(false); }
    SignalOwnership(SignalOwnership const &) = delete;
    SignalOwnership &operator=(SignalOwnership const &) = delete;

  private:
    static std::atomic<bool> &active() {
        static std::atomic<bool> value{false};
        return value;
    }
};

} // namespace

struct SignalStopState {
    explicit SignalStopState(std::function<void()> request_) : request{std::move(request_)} {
        sigemptyset(&signals);
        sigaddset(&signals, SIGINT);
        sigaddset(&signals, SIGTERM);
        auto const result = pthread_sigmask(SIG_BLOCK, &signals, &previous);
        if (result != 0)
            throw std::system_error{result, std::generic_category(), "blocking stop signals"};
        try {
            waiter = std::thread{[this] { wait(); }};
        } catch (...) {
            if (pthread_sigmask(SIG_SETMASK, &previous, nullptr) != 0)
                std::terminate();
            throw;
        }
    }
    ~SignalStopState() {
        if (owner != std::this_thread::get_id())
            std::terminate();
        closing.store(true);
        waiter.join();
        if (pthread_sigmask(SIG_SETMASK, &previous, nullptr) != 0)
            std::terminate();
    }
    SignalStopState(SignalStopState const &) = delete;
    SignalStopState &operator=(SignalStopState const &) = delete;
    void wait() noexcept {
        while (not closing.load()) {
            auto const timeout = timespec{0, 50'000'000};
            auto const signal = sigtimedwait(&signals, nullptr, &timeout);
            if (signal == SIGINT or signal == SIGTERM)
                request();
            else if (signal < 0 and errno != EAGAIN and errno != EINTR) {
                failure.store(errno);
                request();
                return;
            }
        }
    }

    SignalOwnership ownership;
    std::function<void()> request;
    std::thread::id owner = std::this_thread::get_id();
    sigset_t signals{}, previous{};
    std::atomic<bool> closing{false};
    std::atomic<int> failure{0};
    std::thread waiter;
};

} // namespace lrclexec::detail

namespace lrclexec {

SignalStop::SignalStop(std::function<void()> requestStop)
    : state{std::make_unique<detail::SignalStopState>(std::move(requestStop))} {}
SignalStop::~SignalStop() = default;
std::error_code SignalStop::error() const noexcept {
    return {state->failure.load(), std::generic_category()};
}

} // namespace lrclexec
