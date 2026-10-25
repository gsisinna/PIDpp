#pragma once

#include <cmath>
#include <concepts>
#include <numbers>

namespace pidpp {

/**
 * Wraps `value` into the half-open interval [lower, upper).
 *
 * Useful for continuous inputs such as joint angles, wheel headings, or
 * encoder counts that roll over. If the range is empty or not finite the
 * value is returned unchanged.
 */
template <std::floating_point T> [[nodiscard]] T wrap_to_range(T value, T lower, T upper) noexcept {
    const T span = upper - lower;
    if (!(span > T{0}) || !std::isfinite(span) || !std::isfinite(value)) {
        return value;
    }
    T wrapped = std::fmod(value - lower, span);
    if (wrapped < T{0}) {
        wrapped += span;
    }
    return wrapped + lower;
}

/**
 * Wraps an angle in radians into [-pi, pi).
 */
template <std::floating_point T> [[nodiscard]] T wrap_angle(T radians) noexcept {
    return wrap_to_range(radians, -std::numbers::pi_v<T>, std::numbers::pi_v<T>);
}

/**
 * Shortest signed angular distance from `from` to `to`, in [-pi, pi).
 */
template <std::floating_point T> [[nodiscard]] T angle_difference(T to, T from) noexcept {
    return wrap_angle(to - from);
}

template <std::floating_point T> [[nodiscard]] constexpr T deg_to_rad(T degrees) noexcept {
    return degrees * std::numbers::pi_v<T> / T{180};
}

template <std::floating_point T> [[nodiscard]] constexpr T rad_to_deg(T radians) noexcept {
    return radians * T{180} / std::numbers::pi_v<T>;
}

} // namespace pidpp
