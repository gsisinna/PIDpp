#pragma once

#include <concepts>

#include <pidpp/core/numeric.hpp>

namespace pidpp {

/**
 * Lightweight first-order plant model for examples and offline tests.
 */
template <std::floating_point T> class FirstOrderPlant {
  public:
    constexpr FirstOrderPlant(T gain, T time_constant, T initial_state = T{0}) noexcept
        : gain_{gain}, time_constant_{time_constant}, state_{initial_state} {}

    constexpr T update(T input, T dt) noexcept {
        if (!is_finite(dt) || dt <= T{0} || time_constant_ <= T{0}) {
            return state_;
        }
        const T derivative = ((gain_ * input) - state_) / time_constant_;
        state_ += derivative * dt;
        return state_;
    }

    constexpr void reset(T state = T{0}) noexcept {
        state_ = state;
    }

    [[nodiscard]] constexpr T state() const noexcept {
        return state_;
    }

  private:
    T gain_{1};
    T time_constant_{1};
    T state_{0};
};

} // namespace pidpp
