#pragma once

#include <concepts>

#include <pidpp/core/numeric.hpp>

namespace pidpp {

/**
 * Limits how fast a signal may change, in units per second.
 *
 * Typical uses: smoothing joystick or teleoperation setpoints, protecting
 * gearboxes from torque steps, and limiting the acceleration of wheel speed
 * commands. Rising and falling rates can differ (e.g. gentle acceleration,
 * firm braking). A non-positive rate disables limiting in that direction.
 */
template <std::floating_point T> class SlewRateLimiter {
  public:
    constexpr SlewRateLimiter() noexcept = default;

    explicit constexpr SlewRateLimiter(T rate, T initial_value = T{0}) noexcept
        : rising_rate_{rate}, falling_rate_{rate}, value_{initial_value} {}

    constexpr SlewRateLimiter(T rising_rate, T falling_rate, T initial_value) noexcept
        : rising_rate_{rising_rate}, falling_rate_{falling_rate}, value_{initial_value} {}

    [[nodiscard]] T update(T input, T dt) noexcept {
        if (!is_finite(input) || !is_finite(dt) || dt <= T{0}) {
            return value_;
        }
        const T delta = input - value_;
        T limited = delta;
        if (rising_rate_ > T{0} && delta > rising_rate_ * dt) {
            limited = rising_rate_ * dt;
        } else if (falling_rate_ > T{0} && delta < -falling_rate_ * dt) {
            limited = -falling_rate_ * dt;
        }
        value_ += limited;
        return value_;
    }

    constexpr void reset(T value = T{0}) noexcept {
        value_ = value;
    }

    [[nodiscard]] constexpr T value() const noexcept {
        return value_;
    }

    [[nodiscard]] constexpr T rising_rate() const noexcept {
        return rising_rate_;
    }

    [[nodiscard]] constexpr T falling_rate() const noexcept {
        return falling_rate_;
    }

  private:
    T rising_rate_{0};
    T falling_rate_{0};
    T value_{0};
};

} // namespace pidpp
