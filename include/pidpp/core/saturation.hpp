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

} // namespace pidpp
