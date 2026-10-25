#pragma once

#include <cmath>
#include <concepts>
#include <cstddef>
#include <span>

#include <pidpp/core/numeric.hpp>

namespace pidpp {

/**
 * Offline metrics for step-response analysis.
 *
 * Overshoot, rise time, and the settling band are measured relative to the
 * step size `target - initial_value`, so steps down and steps that do not
 * start at zero are handled correctly.
 */
template <std::floating_point T> struct StepResponseMetrics {
    T final_value{0};
    /// Extreme value in the direction of the step.
    T peak_value{0};
    T overshoot_percent{0};
    /// 10 % to 90 % rise time. Only meaningful when `rose` is true.
    T rise_time{0};
    /// Time from the first sample until the response stays inside the band.
    T settling_time{0};
    bool settled{false};
    /// True when the response reached 90 % of the step.
    bool rose{false};
    /// target - final_value.
    T steady_state_error{0};
    /// Integral of absolute error (trapezoidal), a common tuning cost.
    T integral_absolute_error{0};
    /// Integral of squared error (trapezoidal).
    T integral_squared_error{0};
};

/**
 * Options for analyze_step_response().
 */
template <std::floating_point T> struct StepResponseOptions {
    T initial_value{0};
    T target{1};
    /// Settling band as a fraction of the step size (0.02 = 2 %).
    T settling_band{0.02};
};

template <std::floating_point T>
[[nodiscard]] StepResponseMetrics<T>
analyze_step_response(std::span<const T> time, std::span<const T> output,
                      const StepResponseOptions<T>& options) noexcept {
    StepResponseMetrics<T> metrics{};
    const T step = options.target - options.initial_value;
    if (time.empty() || output.empty() || time.size() != output.size() || step == T{0} ||
        !is_finite(step) || !is_finite(options.settling_band) || options.settling_band < T{0}) {
        return metrics;
    }

    const auto normalized = [&](std::size_t i) {
        return (output[i] - options.initial_value) / step;
    };

    metrics.final_value = output.back();
    metrics.steady_state_error = options.target - metrics.final_value;

    T peak = normalized(0);
    std::size_t peak_index = 0;
    for (std::size_t i = 1; i < output.size(); ++i) {
        if (normalized(i) > peak) {
            peak = normalized(i);
            peak_index = i;
        }
    }
    metrics.peak_value = output[peak_index];
    metrics.overshoot_percent = peak > T{1} ? (peak - T{1}) * T{100} : T{0};

    bool crossed_10 = false;
    T t10 = time.front();
    for (std::size_t i = 0; i < output.size(); ++i) {
        const T value = normalized(i);
        if (!crossed_10 && value >= T{0.1}) {
            crossed_10 = true;
            t10 = time[i];
        }
        if (crossed_10 && value >= T{0.9}) {
            metrics.rise_time = time[i] - t10;
            metrics.rose = true;
            break;
        }
    }

    // Settling: scan backwards for the last sample outside the band. O(n).
    const T band = std::abs(step) * options.settling_band;
    std::size_t first_settled = 0;
    bool last_sample_inside = true;
    for (std::size_t i = output.size(); i-- > 0;) {
        if (std::abs(output[i] - options.target) > band) {
            first_settled = i + 1;
            last_sample_inside = i + 1 < output.size();
            break;
        }
    }
    if (last_sample_inside) {
        metrics.settled = true;
        metrics.settling_time = time[first_settled] - time.front();
    }

    for (std::size_t i = 1; i < output.size(); ++i) {
        const T dt = time[i] - time[i - 1];
        const T e0 = options.target - output[i - 1];
        const T e1 = options.target - output[i];
        metrics.integral_absolute_error += T{0.5} * (std::abs(e0) + std::abs(e1)) * dt;
        metrics.integral_squared_error += T{0.5} * ((e0 * e0) + (e1 * e1)) * dt;
    }

    return metrics;
}

/**
 * Step response from rest at zero toward `target` (backward-compatible overload).
 */
template <std::floating_point T>
[[nodiscard]] StepResponseMetrics<T> analyze_step_response(std::span<const T> time,
                                                           std::span<const T> output, T target,
                                                           T settling_band = T{0.02}) noexcept {
    return analyze_step_response(time, output, StepResponseOptions<T>{T{0}, target, settling_band});
}

/**
 * Metrics for trajectory tracking (reference varies over time).
 */
template <std::floating_point T> struct TrackingMetrics {
    T rms_error{0};
    T max_abs_error{0};
    T mean_abs_error{0};
};

template <std::floating_point T>
[[nodiscard]] TrackingMetrics<T> analyze_tracking(std::span<const T> reference,
                                                  std::span<const T> measurement) noexcept {
    TrackingMetrics<T> metrics{};
    if (reference.empty() || reference.size() != measurement.size()) {
        return metrics;
    }
    T sum_sq{0};
    T sum_abs{0};
    for (std::size_t i = 0; i < reference.size(); ++i) {
        const T error = std::abs(reference[i] - measurement[i]);
        sum_sq += error * error;
        sum_abs += error;
        if (error > metrics.max_abs_error) {
            metrics.max_abs_error = error;
        }
    }
    const T count = static_cast<T>(reference.size());
    metrics.rms_error = std::sqrt(sum_sq / count);
    metrics.mean_abs_error = sum_abs / count;
    return metrics;
}

} // namespace pidpp
