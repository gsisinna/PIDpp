#pragma once

#include <cmath>
#include <concepts>

#include <pidpp/core/numeric.hpp>
#include <pidpp/core/result.hpp>

namespace pidpp {

/**
 * Kinematic limits for a motion profile.
 */
template <std::floating_point T> struct MotionConstraints {
    T max_velocity{1};
    T max_acceleration{1};

    [[nodiscard]] Result validate() const noexcept {
        if (!is_finite(max_velocity) || max_velocity <= T{0}) {
            return Result::error(ErrorCode::InvalidConfiguration, "max velocity must be positive");
        }
        if (!is_finite(max_acceleration) || max_acceleration <= T{0}) {
            return Result::error(ErrorCode::InvalidConfiguration,
                                 "max acceleration must be positive");
        }
        return Result::ok();
    }
};

/**
 * Reference sample produced by a motion profile.
 */
template <std::floating_point T> struct MotionState {
    T position{0};
    T velocity{0};
    T acceleration{0};
};

/**
 * Rest-to-rest trapezoidal velocity profile.
 *
 * Generates position, velocity, and acceleration references for point-to-point
 * moves of joints, linear axes, or wheels. Falls back to a triangular profile
 * when the distance is too short to reach the velocity limit. The velocity and
 * acceleration samples are meant to drive PID feedforward terms.
 *
 * plan() runs in constant time and sample() is a pure O(1) function of time,
 * so both are safe to call from a real-time loop.
 */
template <std::floating_point T> class TrapezoidalProfile {
  public:
    constexpr TrapezoidalProfile() noexcept = default;

    /**
     * Plans a move from `start` to `goal`. On failure the previous plan is
     * kept and the error is returned.
     */
    [[nodiscard]] Result plan(T start, T goal, const MotionConstraints<T>& constraints) noexcept {
        if (const auto valid = constraints.validate(); !valid) {
            return valid;
        }
        if (!is_finite(start) || !is_finite(goal)) {
            return Result::error(ErrorCode::InvalidConfiguration,
                                 "profile endpoints must be finite");
        }

        start_ = start;
        goal_ = goal;
        direction_ = goal >= start ? T{1} : T{-1};
        acceleration_ = constraints.max_acceleration;

        const T distance = std::abs(goal - start);
        T accel_time = constraints.max_velocity / acceleration_;
        T accel_distance = T{0.5} * acceleration_ * accel_time * accel_time;

        if (T{2} * accel_distance > distance) {
            // Triangular profile: the velocity limit is never reached.
            accel_time = std::sqrt(distance / acceleration_);
            accel_distance = T{0.5} * distance;
            cruise_time_ = T{0};
        } else {
            cruise_time_ = (distance - (T{2} * accel_distance)) / constraints.max_velocity;
        }

        accel_time_ = accel_time;
        accel_distance_ = accel_distance;
        peak_velocity_ = acceleration_ * accel_time;
        duration_ = (T{2} * accel_time_) + cruise_time_;
        return Result::ok();
    }

    /// Returns the reference at time `t` seconds after the start of the move.
    [[nodiscard]] constexpr MotionState<T> sample(T t) const noexcept {
        if (t <= T{0}) {
            return {start_, T{0}, T{0}};
        }
        if (t >= duration_) {
            return {goal_, T{0}, T{0}};
        }
        if (t < accel_time_) {
            return {start_ + (direction_ * T{0.5} * acceleration_ * t * t),
                    direction_ * acceleration_ * t, direction_ * acceleration_};
        }
        if (t < accel_time_ + cruise_time_) {
            return {start_ +
                        (direction_ * (accel_distance_ + (peak_velocity_ * (t - accel_time_)))),
                    direction_ * peak_velocity_, T{0}};
        }
        const T remaining = duration_ - t;
        return {goal_ - (direction_ * T{0.5} * acceleration_ * remaining * remaining),
                direction_ * acceleration_ * remaining, -direction_ * acceleration_};
    }

    [[nodiscard]] constexpr bool finished(T t) const noexcept {
        return t >= duration_;
    }

    [[nodiscard]] constexpr T duration() const noexcept {
        return duration_;
    }

    [[nodiscard]] constexpr T peak_velocity() const noexcept {
        return peak_velocity_;
    }

    [[nodiscard]] constexpr T start() const noexcept {
        return start_;
    }

    [[nodiscard]] constexpr T goal() const noexcept {
        return goal_;
    }

  private:
    T start_{0};
    T goal_{0};
    T direction_{1};
    T acceleration_{1};
    T accel_time_{0};
    T accel_distance_{0};
    T cruise_time_{0};
    T peak_velocity_{0};
    T duration_{0};
};

} // namespace pidpp
