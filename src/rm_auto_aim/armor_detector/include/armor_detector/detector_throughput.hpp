#ifndef ARMOR_DETECTOR__DETECTOR_THROUGHPUT_HPP_
#define ARMOR_DETECTOR__DETECTOR_THROUGHPUT_HPP_

#include <array>
#include <chrono>
#include <cstddef>
#include <cstdint>

namespace rm_auto_aim
{
// Caller serializes updates/sample with the node's processing_mutex_.
class DetectorThroughput
{
public:
    using Clock = std::chrono::steady_clock;
    enum Counter : std::size_t { INPUT, ACCEPTED, COMPLETED, PUBLISHED, REJECTED, FAILED, DISCARDED, COUNT };
    using Counts = std::array<std::uint64_t, COUNT>;
    struct Sample
    {
        double seconds = 0.0;
        std::array<double, COUNT> fps{};
        Counts totals{};
    };

    explicit DetectorThroughput(Clock::time_point start = Clock::now()) : start_(start) {}
    void add(Counter counter) { ++totals_[counter]; }
    Sample sample(Clock::time_point now = Clock::now())
    {
        Sample result;
        result.seconds = std::chrono::duration<double>(now - start_).count();
        result.totals = totals_;
        // Do not consume counts on a nonpositive interval (also makes deterministic tests safe).
        if (result.seconds <= 0.0) { result.seconds = 0.0; return result; }
        for (std::size_t i = 0; i < COUNT; ++i)
        {
            result.fps[i] = static_cast<double>(totals_[i] - previous_[i]) / result.seconds;
        }
        previous_ = totals_;
        start_ = now;
        return result;
    }

private:
    Clock::time_point start_;
    Counts totals_{};
    Counts previous_{};
};
}  // namespace rm_auto_aim

#endif  // ARMOR_DETECTOR__DETECTOR_THROUGHPUT_HPP_
