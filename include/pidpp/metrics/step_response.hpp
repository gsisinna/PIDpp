#pragma once

#include <concepts>
#include <cstddef>
#include <span>

#include <pidpp/core/numeric.hpp>

namespace pidpp {

/**
 * Offline metrics for step-response analysis.
 */
template <std::floating_point T> struct StepResponseMetrics {
    T final_value{0};
    T peak_value{0};
    T overshoot_percent{0};
    T rise_time{0};
    T settling_time{0};
    bool settled{false};
};

template <std::floating_point T>
[[nodiscard]] StepResponseMetrics<T> analyze_step_response(std::span<const T> time,
                                                           std::span<const T> output, T target,
                                                           T settling_band = T{0.02}) noexcept {
    StepResponseMetrics<T> metrics{};
    if (time.empty() || output.empty() || time.size() != output.size() || target == T{0}) {
        return metrics;
    }

    metrics.final_value = output.back();
    metrics.peak_value = output.front();
    for (const T value : output) {
        if (value > metrics.peak_value) {
            metrics.peak_value = value;
        }
    }

    metrics.overshoot_percent = ((metrics.peak_value - target) / target) * T{100};
    if (metrics.overshoot_percent < T{0}) {
        metrics.overshoot_percent = T{0};
    }

    bool crossed_10 = false;
    T t10 = time.front();
    T t90 = time.front();
    for (std::size_t i = 0; i < output.size(); ++i) {
        if (!crossed_10 && output[i] >= target * T{0.1}) {
            crossed_10 = true;
            t10 = time[i];
        }
        if (crossed_10 && output[i] >= target * T{0.9}) {
            t90 = time[i];
            break;
        }
    }
    metrics.rise_time = t90 - t10;

    const T lower = target * (T{1} - settling_band);
    const T upper = target * (T{1} + settling_band);
    for (std::size_t i = 0; i < output.size(); ++i) {
        bool remains_settled = true;
        for (std::size_t j = i; j < output.size(); ++j) {
            if (output[j] < lower || output[j] > upper) {
                remains_settled = false;
                break;
            }
        }
        if (remains_settled) {
            metrics.settling_time = time[i] - time.front();
            metrics.settled = true;
            break;
        }
    }

    return metrics;
}

} // namespace pidpp
