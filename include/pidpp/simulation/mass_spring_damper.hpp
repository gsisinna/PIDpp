#pragma once

#include <concepts>

#include <pidpp/core/numeric.hpp>

namespace pidpp {

/**
 * Simple mass-spring-damper simulation model.
 */
template <std::floating_point T> class MassSpringDamper {
  public:
    constexpr MassSpringDamper(T mass, T damping, T stiffness) noexcept
        : mass_{mass}, damping_{damping}, stiffness_{stiffness} {}

    constexpr T update(T force, T dt) noexcept {
        if (!is_finite(dt) || dt <= T{0} || mass_ <= T{0}) {
            return position_;
        }
        const T acceleration = (force - (damping_ * velocity_) - (stiffness_ * position_)) / mass_;
        velocity_ += acceleration * dt;
        position_ += velocity_ * dt;
        return position_;
    }

    constexpr void reset(T position = T{0}, T velocity = T{0}) noexcept {
        position_ = position;
        velocity_ = velocity;
    }

    [[nodiscard]] constexpr T position() const noexcept {
        return position_;
    }

    [[nodiscard]] constexpr T velocity() const noexcept {
        return velocity_;
    }

  private:
    T mass_{1};
    T damping_{0};
    T stiffness_{0};
    T position_{0};
    T velocity_{0};
};

} // namespace pidpp
