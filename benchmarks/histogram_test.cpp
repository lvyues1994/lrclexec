#include "Metrics.h"
#include <cmath>
#include <random>
#include <stdexcept>
#include <vector>

int main() {
    auto histogram = measurement::Histogram{};
    auto values = std::vector<std::uint64_t>{0, 1, 2, 15, 16, 17, 31, 32, 1023, 1024, std::uint64_t{1} << 50};
    auto random = std::mt19937_64{42};
    for (int i = 0; i < 10000; ++i)
        values.push_back(random() % 1000000000);
    for (auto const value : values)
        histogram.add(value);
    std::sort(values.begin(), values.end());
    for (auto const fraction : {0.01, 0.50, 0.95, 0.99, 1.0}) {
        auto const index =
            static_cast<std::size_t>(std::ceil(fraction * static_cast<double>(values.size()))) - 1;
        auto const actual = values.at(index);
        auto const upper = histogram.percentile(fraction);
        if (upper < actual or upper > actual + actual / 16 + 2)
            throw std::runtime_error{"histogram percentile disagrees with sorted oracle"};
    }
    return histogram.count() == values.size() and histogram.maximum() == values.back() ? 0 : 1;
}
