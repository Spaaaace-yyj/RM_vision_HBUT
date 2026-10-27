#include "armor_detector/detector_throughput.hpp"

#include <cmath>
#include <iostream>
#include <stdexcept>

namespace
{
void require(bool value, const char * message)
{
    if (!value) { throw std::runtime_error(message); }
}
void near(double actual, double expected)
{
    require(std::abs(actual - expected) < 1e-9, "incorrect rate/window duration");
}
}  // namespace

int main()
{
    using Counter = rm_auto_aim::DetectorThroughput;
    using namespace std::chrono_literals;
    try
    {
        const auto epoch = Counter::Clock::time_point{};
        Counter rates(epoch);
        for (int i = 0; i < 206; ++i) { rates.add(Counter::INPUT); }
        for (int i = 0; i < 104; ++i) { rates.add(Counter::REJECTED); }
        for (int i = 0; i < 102; ++i) { rates.add(Counter::ACCEPTED); }
        // A result may cross the timer boundary. Do not equate input/accepted/completed.
        for (int i = 0; i < 101; ++i) { rates.add(Counter::COMPLETED); rates.add(Counter::PUBLISHED); }
        auto sample = rates.sample(epoch);  // zero interval must not consume counters
        near(sample.seconds, 0.0);
        sample = rates.sample(epoch + 2s);
        near(sample.seconds, 2.0);
        near(sample.fps[Counter::INPUT], 103.0);
        near(sample.fps[Counter::REJECTED], 52.0);
        near(sample.fps[Counter::ACCEPTED], 51.0);
        near(sample.fps[Counter::COMPLETED], 50.5);
        require(sample.totals[Counter::INPUT] == 206, "totals reset with rate window");
        rates.add(Counter::COMPLETED);
        rates.add(Counter::PUBLISHED);
        rates.add(Counter::FAILED);  // errors can coexist with successful fallback completions
        rates.add(Counter::DISCARDED);
        sample = rates.sample(epoch + 3500ms);
        near(sample.seconds, 1.5);
        near(sample.fps[Counter::INPUT], 0.0);
        near(sample.fps[Counter::COMPLETED], 1.0 / 1.5);
        near(sample.fps[Counter::PUBLISHED], 1.0 / 1.5);
        near(sample.fps[Counter::FAILED], 1.0 / 1.5);
        near(sample.fps[Counter::DISCARDED], 1.0 / 1.5);
        require(sample.totals[Counter::COMPLETED] == 102, "wrong cumulative completed count");
        sample = rates.sample(epoch + 4500ms);
        for (double rate : sample.fps) { near(rate, 0.0); }
        require(sample.totals[Counter::INPUT] == 206, "idle timer must retain totals");
        std::cout << "detector_throughput: passed (actual intervals, independent counters, idle windows)\n";
        return 0;
    }
    catch (const std::exception & error)
    {
        std::cerr << error.what() << '\n';
        return 1;
    }
}
