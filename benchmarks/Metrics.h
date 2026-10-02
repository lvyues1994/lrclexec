#pragma once

#include <algorithm>
#include <array>
#include <atomic>
#include <chrono>
#include <cmath>
#include <condition_variable>
#include <cstdint>
#include <fstream>
#include <memory>
#include <mutex>
#include <optional>
#include <string>

namespace measurement {
using Clock = std::chrono::steady_clock;
enum struct Kind { post, schedule, timerCancel, timerRace, service, topic, count };
inline constexpr std::array<char const *, 6> names{"post",       "schedule", "timer_cancel",
                                                   "timer_race", "service",  "topic"};

// Fixed storage: 16 buckets per power of two; percentiles report bucket upper bounds.
struct Histogram {
    void add(std::uint64_t const ns) noexcept {
        auto exponent = 0u;
        for (auto value = ns; value > 1; value >>= 1)
            ++exponent;
        auto const base = std::uint64_t{1} << exponent;
        auto const remainder = ns > base ? ns - base : 0;
        auto const part = exponent >= 4 ? remainder >> (exponent - 4) : remainder << (4 - exponent);
        ++buckets[exponent * 16 + part];
        ++sampleCount;
        if (ns > maxValue)
            maxValue = ns;
    }
    std::uint64_t percentile(double const fraction) const noexcept {
        if (sampleCount == 0)
            return 0;
        auto const rank = static_cast<std::uint64_t>(std::ceil(fraction * static_cast<double>(sampleCount)));
        auto accumulated = std::uint64_t{0};
        for (std::size_t i = 0; i < buckets.size(); ++i) {
            accumulated += buckets[i];
            if (accumulated >= rank) {
                auto const exponent = i / 16;
                // Long experimental waits still fit this saturated integer bound.
                if (exponent == 63)
                    return UINT64_MAX;
                auto const base = std::uint64_t{1} << exponent;
                auto const part = i % 16 + 1;
                return base + (exponent >= 4 ? part << (exponent - 4) : (part * base + 15) / 16);
            }
        }
        return maxValue;
    }
    std::uint64_t count() const noexcept { return sampleCount; }
    std::uint64_t maximum() const noexcept { return maxValue; }

  private:
    std::array<std::uint64_t, 1024> buckets{};
    std::uint64_t sampleCount = 0, maxValue = 0;
};
struct Totals {
    std::uint64_t admitted = 0, values = 0, stopped = 0, errors = 0, duplicates = 0;
    std::uint64_t inflight = 0, peak = 0;
    std::array<Histogram, 6> latency{};
};
struct Ticket {
    Kind kind;
    Clock::time_point start;
    std::shared_ptr<std::atomic<bool>> finished;
};

// Owns the accounting invariant and the admission limit, not the library queue.
struct Accounting {
    explicit Accounting(std::uint64_t const limit) : limit{limit} {}
    std::optional<Ticket> admit(Kind const kind, Clock::time_point const deadline) {
        auto flag = std::make_shared<std::atomic<bool>>(false);
        auto lock = std::unique_lock<std::mutex>{mutex};
        if (not available.wait_until(lock, deadline, [&] { return failed or totals.inflight < limit; }) or
            failed or Clock::now() >= deadline)
            return {};
        ++totals.admitted;
        ++totals.inflight;
        totals.peak = std::max(totals.peak, totals.inflight);
        return Ticket{kind, Clock::now(), std::move(flag)};
    }
    void finish(Ticket const &ticket, int const outcome,
                Clock::time_point const end = Clock::now()) noexcept {
        auto const lock = std::lock_guard<std::mutex>{mutex};
        if (ticket.finished->exchange(true)) {
            ++totals.duplicates;
            failed = true;
        } else {
            if (outcome > 0)
                ++totals.values;
            else if (outcome == 0)
                ++totals.stopped;
            else {
                ++totals.errors;
                failed = true;
            }
            auto const ns = std::chrono::duration_cast<std::chrono::nanoseconds>(end - ticket.start).count();
            totals.latency[static_cast<std::size_t>(ticket.kind)].add(static_cast<std::uint64_t>(ns));
            --totals.inflight;
        }
        available.notify_all();
    }
    Totals snapshot() const {
        auto const lock = std::lock_guard<std::mutex>{mutex};
        return totals;
    }
    bool failedRun() const {
        auto const lock = std::lock_guard<std::mutex>{mutex};
        return failed;
    }
    void fail() noexcept {
        auto const lock = std::lock_guard<std::mutex>{mutex};
        failed = true;
        available.notify_all();
    }

  private:
    std::uint64_t limit;
    mutable std::mutex mutex;
    std::condition_variable available;
    Totals totals;
    bool failed = false;
};

inline std::optional<std::uint64_t> residentKiB() {
    auto input = std::ifstream{"/proc/self/status"};
    for (std::string line; std::getline(input, line);)
        if (line.rfind("VmRSS:", 0) == 0)
            return std::stoull(line.substr(6));
    return {};
}
} // namespace measurement
