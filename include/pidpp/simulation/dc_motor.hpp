#pragma once

#include <concepts>

#include <pidpp/core/numeric.hpp>

namespace pidpp {

/**
 * Minimal brushed DC motor model for SDK examples and deterministic tests.
 */
template <std::floating_point T> class DcMotor {
  public:
    constexpr DcMotor(T resistance, T inductance, T torque_constant, T back_emf_constant, T inertia,
                      T damping) noexcept
        : resistance_{resistance}, inductance_{inductance}, torque_constant_{torque_constant},
          back_emf_constant_{back_emf_constant}, inertia_{inertia}, damping_{damping} {}

    [[nodiscard]] constexpr T update(T voltage, T load_torque, T dt) noexcept {
        if (!is_finite(dt) || dt <= T{0} || inductance_ <= T{0} || inertia_ <= T{0}) {
            return angular_velocity_;
        }

        const T current_derivative =
            (voltage - (resistance_ * current_) - (back_emf_constant_ * angular_velocity_)) /
            inductance_;
        current_ += current_derivative * dt;

        const T motor_torque = torque_constant_ * current_;
        const T acceleration =
            (motor_torque - load_torque - (damping_ * angular_velocity_)) / inertia_;
        angular_velocity_ += acceleration * dt;
        position_ += angular_velocity_ * dt;
        return angular_velocity_;
    }

    constexpr void reset() noexcept {
        current_ = T{0};
        angular_velocity_ = T{0};
        position_ = T{0};
    }

    [[nodiscard]] constexpr T current() const noexcept {
        return current_;
    }

    [[nodiscard]] constexpr T angular_velocity() const noexcept {
        return angular_velocity_;
    }

    [[nodiscard]] constexpr T position() const noexcept {
        return position_;
    }

  private:
    T resistance_{1};
    T inductance_{1};
    T torque_constant_{1};
    T back_emf_constant_{1};
    T inertia_{1};
    T damping_{0};
    T current_{0};
    T angular_velocity_{0};
    T position_{0};
};

} // namespace pidpp
