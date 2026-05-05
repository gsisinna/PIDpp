#pragma once

#include <cmath>
#include <concepts>
#include <limits>

namespace pidpp {

template <std::floating_point T> [[nodiscard]] constexpr bool is_positive(T value) noexcept {
    return value > T{0};
}

template <std::floating_point T> [[nodiscard]] bool is_finite(T value) noexcept {
    return std::isfinite(value);
}

template <std::floating_point T> [[nodiscard]] constexpr T epsilon() noexcept {
    return std::numeric_limits<T>::epsilon();
}

} // namespace pidpp
