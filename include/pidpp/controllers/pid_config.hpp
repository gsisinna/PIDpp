#pragma once

#include <concepts>

#include <pidpp/controllers/anti_windup.hpp>
#include <pidpp/controllers/derivative_mode.hpp>
#include <pidpp/controllers/feedforward.hpp>
#include <pidpp/core/numeric.hpp>
#include <pidpp/core/result.hpp>

namespace pidpp {

/**
 * PID controller configuration.
 *
 * Validate this object before using it in a control loop. Validation is
 * explicit and non-throwing; the controller update path remains noexcept and
 * allocation-free.
 */
template <std::floating_point T>
struct PidConfig {
    T kp{0};
    T ki{0};
    T kd{0};
    T output_min{-1};
    T output_max{1};
    T integral_min{-1};
    T integral_max{1};
    AntiWindupStrategy anti_windup{AntiWindupStrategy::None};
    DerivativeMode derivative_mode{DerivativeMode::OnError};
    T derivative_filter_tau{0};
    T back_calculation_gain{1};
    FeedforwardConfig<T> feedforward{};

    [[nodiscard]] Result validate() const noexcept
    {
        if (!is_finite(kp) || !is_finite(ki) || !is_finite(kd)) {
            return Result::error(ErrorCode::InvalidGain, "PID gains must be finite");
        }
        if (kp < T{0} || ki < T{0} || kd < T{0}) {
            return Result::error(ErrorCode::InvalidGain, "PID gains must be non-negative");
        }
        if (!is_finite(output_min) || !is_finite(output_max) || output_min >= output_max) {
            return Result::error(ErrorCode::InvalidOutputLimits, "output range is invalid");
        }
        if (!is_finite(integral_min) || !is_finite(integral_max) || integral_min > integral_max) {
            return Result::error(ErrorCode::InvalidConfiguration, "integral range is invalid");
        }
        if (!is_finite(derivative_filter_tau) || derivative_filter_tau < T{0}) {
            return Result::error(ErrorCode::InvalidFilterTimeConstant, "derivative filter tau must be non-negative");
        }
        if (!is_finite(back_calculation_gain) || back_calculation_gain < T{0}) {
            return Result::error(ErrorCode::InvalidConfiguration, "back calculation gain must be non-negative");
        }
        if (!is_finite(feedforward.velocity_gain) ||
            !is_finite(feedforward.acceleration_gain) ||
            !is_finite(feedforward.static_gain)) {
            return Result::error(ErrorCode::InvalidConfiguration, "feedforward gains must be finite");
        }
        return Result::ok();
    }
};

} // namespace pidpp
