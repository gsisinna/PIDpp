#pragma once

#include <concepts>

#include <pidpp/core/numeric.hpp>

namespace pidpp {

/**
 * First-order low-pass filter with caller-owned state and no allocation.
 */
template <std::floating_point T> class LowPassFilter {
  public:
    constexpr LowPassFilter() noexcept = default;

    explicit constexpr LowPassFilter(T tau) noexcept : tau_{tau} {}

    constexpr void set_tau(T tau) noexcept {
        tau_ = tau;
    }

    [[nodiscard]] constexpr T tau() const noexcept {
        return tau_;
    }

    constexpr void reset() noexcept {
        value_ = T{0};
        initialized_ = false;
    }

    constexpr void reset(T value) noexcept {
        value_ = value;
        initialized_ = true;
    }

    [[nodiscard]] T update(T input, T dt) noexcept {
        if (!is_finite(input) || !is_finite(dt) || dt <= T{0}) {
            return value_;
        }
        if (!initialized_ || tau_ <= T{0}) {
            value_ = input;
            initialized_ = true;
            return value_;
        }

        const T alpha = dt / (tau_ + dt);
        value_ += alpha * (input - value_);
        return value_;
    }

    [[nodiscard]] constexpr T value() const noexcept {
        return value_;
    }

  private:
    T tau_{0};
    T value_{0};
    bool initialized_{false};
};

} // namespace pidpp
