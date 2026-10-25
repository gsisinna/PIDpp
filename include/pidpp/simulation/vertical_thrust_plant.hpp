#pragma once

#include <concepts>

#include <pidpp/core/numeric.hpp>

namespace pidpp {

/**
 * Vertical axis of a multirotor drone (or any thrust-lifted body).
 *
 *     m dv/dt = thrust_actual - m g - c v
 *
 * Thrust reaches the body through a first-order motor/propeller lag. The
 * vehicle cannot sink below the ground (altitude 0), where it rests.
 */
template <std::floating_point T> class VerticalThrustPlant {
  public:
    static constexpr T standard_gravity = T{9.80665};

    /**
     * @param mass            vehicle mass in kg
     * @param drag            linear vertical drag coefficient in N*s/m
     * @param motor_time_constant thrust response lag in s
     */
    constexpr VerticalThrustPlant(T mass, T drag, T motor_time_constant) noexcept
        : mass_{mass}, drag_{drag}, motor_tau_{motor_time_constant} {}

    T update(T thrust_command, T dt) noexcept {
        if (!is_finite(dt) || dt <= T{0} || !is_finite(thrust_command) || mass_ <= T{0}) {
            return altitude_;
        }
        thrust_ = motor_tau_ > T{0}
                      ? thrust_ + ((thrust_command - thrust_) * dt / (motor_tau_ + dt))
                      : thrust_command;

        const T acceleration = (thrust_ - (mass_ * gravity_) - (drag_ * velocity_)) / mass_;
        velocity_ += acceleration * dt;
        altitude_ += velocity_ * dt;
        if (altitude_ <= T{0}) {
            altitude_ = T{0};
            if (velocity_ < T{0}) {
                velocity_ = T{0};
            }
        }
        return altitude_;
    }

    /// Thrust needed to hover (the ideal static feedforward).
    [[nodiscard]] constexpr T hover_thrust() const noexcept {
        return mass_ * gravity_;
    }

    constexpr void set_mass(T mass) noexcept {
        mass_ = mass;
    }

    constexpr void reset(T altitude = T{0}, T velocity = T{0}) noexcept {
        altitude_ = altitude;
        velocity_ = velocity;
        thrust_ = T{0};
    }

    [[nodiscard]] constexpr T altitude() const noexcept {
        return altitude_;
    }

    [[nodiscard]] constexpr T velocity() const noexcept {
        return velocity_;
    }

    [[nodiscard]] constexpr T thrust() const noexcept {
        return thrust_;
    }

    [[nodiscard]] constexpr T mass() const noexcept {
        return mass_;
    }

  private:
    T mass_{1};
    T drag_{0};
    T motor_tau_{0};
    T gravity_{standard_gravity};
    T thrust_{0};
    T altitude_{0};
    T velocity_{0};
};

} // namespace pidpp
