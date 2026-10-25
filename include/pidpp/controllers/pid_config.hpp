#pragma once

#include <concepts>
#include <numbers>

#include <pidpp/controllers/anti_windup.hpp>
#include <pidpp/controllers/derivative_mode.hpp>
#include <pidpp/controllers/feedforward.hpp>
#include <pidpp/core/numeric.hpp>
#include <pidpp/core/result.hpp>

namespace pidpp {

/**
 * Proportional, integral, and derivative gains in parallel form:
 *
 *     u = kp * e + ki * integral(e) + kd * de/dt
 */
template <std::floating_point T> struct PidGains {
    T kp{0};
    T ki{0};
    T kd{0};

    [[nodiscard]] Result validate() const noexcept {
        if (!is_finite(kp) || !is_finite(ki) || !is_finite(kd)) {
            return Result::error(ErrorCode::InvalidGain, "PID gains must be finite");
        }
        if (kp < T{0} || ki < T{0} || kd < T{0}) {
            return Result::error(ErrorCode::InvalidGain, "PID gains must be non-negative");
        }
        return Result::ok();
    }
};

/**
 * PID controller configuration.
 *
 * Validate this object before using it in a control loop. Validation is
 * explicit and non-throwing; the controller update path remains noexcept and
 * allocation-free.
 *
 * Units: `integral_min`/`integral_max` bound the accumulated error
 * (error * seconds), not the integral contribution to the output. The
 * integral contribution is `ki * integral`.
 */
template <std::floating_point T> struct PidConfig {
    T kp{0};
    T ki{0};
    T kd{0};
    T output_min{-1};
    T output_max{1};
    T integral_min{-1};
    T integral_max{1};
    AntiWindupStrategy anti_windup{AntiWindupStrategy::None};
    DerivativeMode derivative_mode{DerivativeMode::OnError};
    /// Time constant of the first-order derivative filter in seconds. 0 disables it.
    T derivative_filter_tau{0};
    /// Tracking gain for AntiWindupStrategy::BackCalculation, in 1/(gain * seconds).
    T back_calculation_gain{1};
    FeedforwardConfig<T> feedforward{};
    /// Errors with magnitude below this width are treated as zero (continuous deadband).
    T error_deadband{0};
    /// Maximum output slew rate in output units per second. 0 disables rate limiting.
    T output_rate_limit{0};
    /// Treat the measurement as a continuous (wrapping) quantity, e.g. an angle.
    bool continuous_input{false};
    T input_range_min{-std::numbers::pi_v<T>};
    T input_range_max{std::numbers::pi_v<T>};

    [[nodiscard]] constexpr PidGains<T> gains() const noexcept {
        return {kp, ki, kd};
    }

    constexpr void set_gains(const PidGains<T>& new_gains) noexcept {
        kp = new_gains.kp;
        ki = new_gains.ki;
        kd = new_gains.kd;
    }

    [[nodiscard]] Result validate() const noexcept {
        if (const auto gain_result = gains().validate(); !gain_result) {
            return gain_result;
        }
        if (!is_finite(output_min) || !is_finite(output_max) || output_min >= output_max) {
            return Result::error(ErrorCode::InvalidOutputLimits, "output range is invalid");
        }
        if (!is_finite(integral_min) || !is_finite(integral_max) || integral_min > integral_max) {
            return Result::error(ErrorCode::InvalidConfiguration, "integral range is invalid");
        }
        if (!is_finite(derivative_filter_tau) || derivative_filter_tau < T{0}) {
            return Result::error(ErrorCode::InvalidFilterTimeConstant,
                                 "derivative filter tau must be non-negative");
        }
        if (!is_finite(back_calculation_gain) || back_calculation_gain < T{0}) {
            return Result::error(ErrorCode::InvalidConfiguration,
                                 "back calculation gain must be non-negative");
        }
        if (!is_finite(feedforward.velocity_gain) || !is_finite(feedforward.acceleration_gain) ||
            !is_finite(feedforward.static_gain) || !is_finite(feedforward.friction_gain)) {
            return Result::error(ErrorCode::InvalidConfiguration,
                                 "feedforward gains must be finite");
        }
        if (!is_finite(error_deadband) || error_deadband < T{0}) {
            return Result::error(ErrorCode::InvalidConfiguration,
                                 "error deadband must be non-negative");
        }
        if (!is_finite(output_rate_limit) || output_rate_limit < T{0}) {
            return Result::error(ErrorCode::InvalidConfiguration,
                                 "output rate limit must be non-negative");
        }
        if (continuous_input && (!is_finite(input_range_min) || !is_finite(input_range_max) ||
                                 input_range_min >= input_range_max)) {
            return Result::error(ErrorCode::InvalidConfiguration,
                                 "continuous input range is invalid");
        }
        return Result::ok();
    }
};

} // namespace pidpp
