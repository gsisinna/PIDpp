#pragma once

#include <array>
#include <concepts>
#include <cstddef>

namespace pidpp {

/**
 * Fixed-size moving average filter.
 *
 * Window storage is embedded in the object; update() does not allocate.
 */
template <std::floating_point T, std::size_t WindowSize>
class MovingAverage {
    static_assert(WindowSize > 0, "WindowSize must be greater than zero");

public:
    constexpr void reset() noexcept
    {
        samples_.fill(T{0});
        sum_ = T{0};
        index_ = 0;
        count_ = 0;
    }

    [[nodiscard]] constexpr T update(T sample) noexcept
    {
        if (count_ < WindowSize) {
            samples_[index_] = sample;
            sum_ += sample;
            ++count_;
        } else {
            sum_ -= samples_[index_];
            samples_[index_] = sample;
            sum_ += sample;
        }

        index_ = (index_ + 1U) % WindowSize;
        return value();
    }

    [[nodiscard]] constexpr T value() const noexcept
    {
        return count_ == 0U ? T{0} : sum_ / static_cast<T>(count_);
    }

    [[nodiscard]] constexpr std::size_t count() const noexcept
    {
        return count_;
    }

private:
    std::array<T, WindowSize> samples_{};
    T sum_{0};
    std::size_t index_{0};
    std::size_t count_{0};
};

} // namespace pidpp
