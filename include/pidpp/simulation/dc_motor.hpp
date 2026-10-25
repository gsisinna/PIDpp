#pragma once

#include <cmath>
#include <concepts>

#include <pidpp/core/numeric.hpp>

namespace pidpp {

/**
 * Minimal brushed DC motor model for SDK examples and deterministic tests.
 *
 *     L di/dt = V - R i - Ke w
 *     J dw/dt = Kt i - tau_load - b w
 *
 * The electrical time constant L/R is usually much shorter than a control
 * period, so update() internally sub-steps the integration to keep the
 * explicit Euler scheme stable for any reasonable dt.
 */
template <std::floating_point T> class DcMotor {
  public:
    constexpr DcMotor(T resistance, T inductance, T torque_constant, T back_emf_constant, T inertia,
                      T damping) noexcept
        : resistance_{resistance}, inductance_{inductance}, torque_constant_{torque_constant},
          back_emf_constant_{back_emf_constant}, inertia_{inertia}, damping_{damping} {}

    T update(T voltage, T load_torque, T dt) noexcept {
        if (!is_finite(dt) || dt <= T{0} || inductance_ <= T{0} || inertia_ <= T{0} ||
            !is_finite(voltage) || !is_finite(load_torque)) {
            return angular_velocity_;
        }

        const int substeps = substep_count(dt);
        const T h = dt / static_cast<T>(substeps);
        for (int i = 0; i < substeps; ++i) {
            step(voltage, load_torque, h);
        }
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

    /// Steady-state no-load speed for a given voltage.
    [[nodiscard]] constexpr T steady_state_speed(T voltage) const noexcept {
        const T denominator = (resistance_ * damping_) + (torque_constant_ * back_emf_constant_);
        return denominator > T{0} ? (torque_constant_ * voltage) / denominator : T{0};
    }

  private:
    static constexpr int max_substeps = 1000;

    [[nodiscard]] int substep_count(T dt) const noexcept {
        // Keep h <= L / (4 R) so the stiff electrical mode stays well damped.
        if (resistance_ <= T{0}) {
            return 1;
        }
        const T max_step = inductance_ / (T{4} * resistance_);
        const T ratio = std::ceil(dt / max_step);
        if (!(ratio > T{1})) {
            return 1;
        }
        return ratio > static_cast<T>(max_substeps) ? max_substeps : static_cast<int>(ratio);
    }

    constexpr void step(T voltage, T load_torque, T h) noexcept {
        const T current_derivative =
            (voltage - (resistance_ * current_) - (back_emf_constant_ * angular_velocity_)) /
            inductance_;
        current_ += current_derivative * h;

        const T motor_torque = torque_constant_ * current_;
        const T acceleration =
            (motor_torque - load_torque - (damping_ * angular_velocity_)) / inertia_;
        angular_velocity_ += acceleration * h;
        position_ += angular_velocity_ * h;
    }

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
