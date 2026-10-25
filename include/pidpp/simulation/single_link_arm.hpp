#pragma once

#include <cmath>
#include <concepts>

#include <pidpp/core/numeric.hpp>

namespace pidpp {

/**
 * Single revolute joint carrying a link under gravity (one-DOF robot arm).
 *
 *     J(m) dw/dt = tau - b w - m g l cos(theta)
 *
 * The angle is measured from the horizontal, so gravity torque is maximal at
 * theta = 0 and zero with the link pointing straight up or down. An optional
 * payload at the tip changes both inertia and gravity load, which makes this
 * model handy for gravity feedforward and gain-scheduling examples.
 */
template <std::floating_point T> class SingleLinkArm {
  public:
    static constexpr T standard_gravity = T{9.80665};

    /**
     * @param link_mass   mass of the link in kg (uniform rod)
     * @param link_length length of the link in m
     * @param damping     viscous joint friction in N*m*s/rad
     * @param rotor_inertia reflected motor/gearbox inertia in kg*m^2
     */
    constexpr SingleLinkArm(T link_mass, T link_length, T damping, T rotor_inertia = T{0}) noexcept
        : link_mass_{link_mass}, link_length_{link_length}, damping_{damping},
          rotor_inertia_{rotor_inertia} {}

    T update(T torque, T dt) noexcept {
        const T inertia = total_inertia();
        if (!is_finite(dt) || dt <= T{0} || !is_finite(torque) || inertia <= T{0}) {
            return angle_;
        }
        const T acceleration = (torque - (damping_ * velocity_) - gravity_torque(angle_)) / inertia;
        velocity_ += acceleration * dt;
        angle_ += velocity_ * dt;
        return angle_;
    }

    /// Torque gravity applies to the joint at `angle` (to be compensated).
    [[nodiscard]] T gravity_torque(T angle) const noexcept {
        const T moment = (link_mass_ * link_length_ * T{0.5}) + (payload_mass_ * link_length_);
        return moment * gravity_ * std::cos(angle);
    }

    [[nodiscard]] constexpr T total_inertia() const noexcept {
        const T link = link_mass_ * link_length_ * link_length_ / T{3};
        const T payload = payload_mass_ * link_length_ * link_length_;
        return link + payload + rotor_inertia_;
    }

    constexpr void set_payload_mass(T mass) noexcept {
        payload_mass_ = mass > T{0} ? mass : T{0};
    }

    constexpr void set_gravity(T gravity) noexcept {
        gravity_ = gravity;
    }

    constexpr void reset(T angle = T{0}, T velocity = T{0}) noexcept {
        angle_ = angle;
        velocity_ = velocity;
    }

    [[nodiscard]] constexpr T angle() const noexcept {
        return angle_;
    }

    [[nodiscard]] constexpr T velocity() const noexcept {
        return velocity_;
    }

    [[nodiscard]] constexpr T payload_mass() const noexcept {
        return payload_mass_;
    }

  private:
    T link_mass_{1};
    T link_length_{1};
    T damping_{0};
    T rotor_inertia_{0};
    T payload_mass_{0};
    T gravity_{standard_gravity};
    T angle_{0};
    T velocity_{0};
};

} // namespace pidpp
