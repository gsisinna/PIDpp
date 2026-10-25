#pragma once

#include <cmath>
#include <concepts>

#include <pidpp/core/angles.hpp>
#include <pidpp/core/numeric.hpp>

namespace pidpp {

/**
 * Planar pose of a mobile robot.
 */
template <std::floating_point T> struct Pose2D {
    T x{0};
    T y{0};
    /// Heading in radians, wrapped to [-pi, pi).
    T heading{0};
};

/**
 * Differential-drive (unicycle) mobile robot kinematics.
 *
 * Commands are body linear velocity v [m/s] and yaw rate w [rad/s]. Each
 * command reaches the robot through a first-order lag that models the wheel
 * speed loops, so a heading controller sees realistic dynamics.
 */
template <std::floating_point T> class DifferentialDrive {
  public:
    /**
     * @param linear_time_constant  lag of the linear velocity response in s
     * @param angular_time_constant lag of the yaw-rate response in s
     */
    constexpr DifferentialDrive(T linear_time_constant, T angular_time_constant) noexcept
        : linear_tau_{linear_time_constant}, angular_tau_{angular_time_constant} {}

    const Pose2D<T>& update(T linear_velocity_command, T angular_velocity_command, T dt) noexcept {
        if (!is_finite(dt) || dt <= T{0} || !is_finite(linear_velocity_command) ||
            !is_finite(angular_velocity_command)) {
            return pose_;
        }
        linear_velocity_ = lag(linear_velocity_, linear_velocity_command, linear_tau_, dt);
        angular_velocity_ = lag(angular_velocity_, angular_velocity_command, angular_tau_, dt);

        // Midpoint integration of the heading for better arc accuracy.
        const T mid_heading = pose_.heading + (T{0.5} * angular_velocity_ * dt);
        pose_.x += linear_velocity_ * std::cos(mid_heading) * dt;
        pose_.y += linear_velocity_ * std::sin(mid_heading) * dt;
        pose_.heading = wrap_angle(pose_.heading + (angular_velocity_ * dt));
        return pose_;
    }

    constexpr void reset(const Pose2D<T>& pose = {}) noexcept {
        pose_ = pose;
        linear_velocity_ = T{0};
        angular_velocity_ = T{0};
    }

    [[nodiscard]] constexpr const Pose2D<T>& pose() const noexcept {
        return pose_;
    }

    [[nodiscard]] constexpr T linear_velocity() const noexcept {
        return linear_velocity_;
    }

    [[nodiscard]] constexpr T angular_velocity() const noexcept {
        return angular_velocity_;
    }

  private:
    [[nodiscard]] static constexpr T lag(T state, T command, T tau, T dt) noexcept {
        if (tau <= T{0}) {
            return command;
        }
        return state + ((command - state) * dt / (tau + dt));
    }

    T linear_tau_{0};
    T angular_tau_{0};
    T linear_velocity_{0};
    T angular_velocity_{0};
    Pose2D<T> pose_{};
};

} // namespace pidpp
