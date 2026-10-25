#pragma once

#include <algorithm>
#include <concepts>

namespace pidpp {

template <std::floating_point T>
[[nodiscard]] constexpr T clamp(T value, T lower, T upper) noexcept {
    return std::min(std::max(value, lower), upper);
}

template <std::floating_point T>
[[nodiscard]] constexpr bool is_saturated(T value, T lower, T upper) noexcept {
    return value <= lower || value >= upper;
}

template <std::floating_point T>
[[nodiscard]] constexpr bool is_outside(T value, T lower, T upper) noexcept {
    return value < lower || value > upper;
}

/**
 * Continuous (shifted) deadband.
 *
 * Values with magnitude up to `width` map to zero; larger values are shifted
 * toward zero by `width`, so the output has no step at the deadband edge.
 * A non-positive width disables the deadband.
 */
template <std::floating_point T>
[[nodiscard]] constexpr T apply_deadband(T value, T width) noexcept {
    if (width <= T{0}) {
        return value;
    }
    if (value > width) {
        return value - width;
    }
    if (value < -width) {
        return value + width;
    }
    return T{0};
}

} // namespace pidpp
