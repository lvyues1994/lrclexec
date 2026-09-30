#include <chrono>
#include <csignal>
#include <iostream>
#include <lrclexec/SignalStop.h>
#include <pthread.h>
#include <stdexcept>
#include <thread>
#include <unistd.h>

int main() {
    auto stop = lexec::inplace_stop_source{};
    sigset_t before{}, after{};
    pthread_sigmask(SIG_SETMASK, nullptr, &before);
    try {
        {
            auto guard = lrclexec::SignalStop{stop};
            auto rejected = false;
            try {
                auto duplicate = lrclexec::SignalStop{stop};
            } catch (std::logic_error const &) {
                rejected = true;
            }
            if (not rejected)
                throw std::runtime_error{"duplicate signal guard accepted"};
            if (kill(getpid(), SIGINT) != 0)
                throw std::runtime_error{"signal delivery failed"};
            auto const deadline = std::chrono::steady_clock::now() + std::chrono::seconds{3};
            while (not stop.stop_requested()) {
                if (std::chrono::steady_clock::now() > deadline)
                    throw std::runtime_error{"signal not consumed"};
                std::this_thread::sleep_for(std::chrono::milliseconds{1});
            }
            if (guard.error())
                throw std::system_error{guard.error()};
        }
        pthread_sigmask(SIG_SETMASK, nullptr, &after);
        for (auto const signal : {SIGINT, SIGTERM, SIGUSR1}) {
            if (sigismember(&before, signal) != sigismember(&after, signal))
                throw std::runtime_error{"signal mask was not restored"};
        }
        { auto next = lrclexec::SignalStop{stop}; }
        return 0;
    } catch (std::exception const &error) {
        std::cerr << error.what() << '\n';
        return 1;
    }
}
