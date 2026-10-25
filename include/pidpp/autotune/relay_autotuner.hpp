#pragma once

#include <cmath>
#include <concepts>
#include <cstdint>
#include <limits>
#include <numbers>

#include <pidpp/controllers/pid_config.hpp>
#include <pidpp/core/numeric.hpp>
#include <pidpp/core/result.hpp>

namespace pidpp {

/**
 * Configuration for the relay (Astrom-Hagglund) autotuning experiment.
 */
template <std::floating_point T> struct RelayAutotuneConfig {
    /// Relay half-amplitude d: the output toggles between bias + d and bias - d.
    T relay_amplitude{1};
    /// Error hysteresis band; use it to reject measurement noise.
    T hysteresis{0};
    /// Output around which the relay toggles (e.g. hover thrust, holding torque).
    T output_bias{0};
    /// Number of consecutive consistent oscillation periods to average.
    std::uint32_t cycles{3};
    /// Relative tolerance for two consecutive periods/amplitudes to count as
    /// the same, settled limit cycle.
    T tolerance{0.05};
    /// Give up after this many seconds when using the timed update. 0 disables.
    T timeout{0};

    [[nodiscard]] Result validate() const noexcept {
        if (!is_finite(relay_amplitude) || relay_amplitude <= T{0}) {
            return Result::error(ErrorCode::InvalidConfiguration,
                                 "relay amplitude must be positive");
        }
        if (!is_finite(hysteresis) || hysteresis < T{0}) {
            return Result::error(ErrorCode::InvalidConfiguration,
                                 "hysteresis must be non-negative");
        }
        if (!is_finite(output_bias)) {
            return Result::error(ErrorCode::InvalidConfiguration, "output bias must be finite");
        }
        if (cycles == 0U) {
            return Result::error(ErrorCode::InvalidConfiguration, "cycles must be at least 1");
        }
        if (!is_finite(tolerance) || tolerance <= T{0}) {
            return Result::error(ErrorCode::InvalidConfiguration, "tolerance must be positive");
        }
        if (!is_finite(timeout) || timeout < T{0}) {
            return Result::error(ErrorCode::InvalidConfiguration, "timeout must be non-negative");
        }
        return Result::ok();
    }
};

template <std::floating_point T> struct RelayAutotuneState {
    T output{0};
    bool high{true};
};

/**
 * Outcome of a relay experiment: ultimate gain Ku and ultimate period Tu.
 */
template <std::floating_point T> struct RelayAutotuneResult {
    T ultimate_gain{0};
    T ultimate_period{0};
    /// Average half peak-to-peak amplitude of the measured oscillation.
    T oscillation_amplitude{0};
    std::uint32_t measured_cycles{0};
    bool complete{false};
    bool timed_out{false};
};

/**
 * Classic tuning rules from (Ku, Tu).
 */
enum class TuningRule {
    ZieglerNicholsP,
    ZieglerNicholsPi,
    ZieglerNicholsPid,
    TyreusLuybenPi,
    TyreusLuybenPid,
    PessenIntegral,
    SomeOvershootPid,
    NoOvershootPid
};

/**
 * Parallel-form PID gains from ultimate gain and period.
 *
 * Ziegler-Nichols gives fast but oscillatory loops; Tyreus-Luyben and the
 * overshoot-limited rules are more conservative and usually a better start
 * for mechanical systems with compliance.
 */
template <std::floating_point T>
[[nodiscard]] constexpr PidGains<T> compute_tuning(T ultimate_gain, T ultimate_period,
                                                   TuningRule rule) noexcept {
    const T ku = ultimate_gain;
    const T tu = ultimate_period;
    if (!(ku > T{0}) || !(tu > T{0})) {
        return {};
    }

    // kp, Ti, Td in standard form, converted to parallel form below.
    T kp{0};
    T ti{0};
    T td{0};
    switch (rule) {
    case TuningRule::ZieglerNicholsP:
        kp = T{0.5} * ku;
        break;
    case TuningRule::ZieglerNicholsPi:
        kp = T{0.45} * ku;
        ti = tu / T{1.2};
        break;
    case TuningRule::ZieglerNicholsPid:
        kp = T{0.6} * ku;
        ti = T{0.5} * tu;
        td = T{0.125} * tu;
        break;
    case TuningRule::TyreusLuybenPi:
        kp = ku / T{3.2};
        ti = T{2.2} * tu;
        break;
    case TuningRule::TyreusLuybenPid:
        kp = ku / T{2.2};
        ti = T{2.2} * tu;
        td = tu / T{6.3};
        break;
    case TuningRule::PessenIntegral:
        kp = T{0.7} * ku;
        ti = T{0.4} * tu;
        td = T{0.15} * tu;
        break;
    case TuningRule::SomeOvershootPid:
        kp = ku / T{3};
        ti = T{0.5} * tu;
        td = tu / T{3};
        break;
    case TuningRule::NoOvershootPid:
        kp = T{0.2} * ku;
        ti = T{0.5} * tu;
        td = tu / T{3};
        break;
    }

    return {kp, ti > T{0} ? kp / ti : T{0}, kp * td};
}

/**
 * Relay feedback autotuner.
 *
 * Replace the PID output with this relay while the plant runs around the
 * setpoint. The relay forces a limit cycle; from its period Tu and amplitude a
 * the ultimate gain is estimated with the describing-function approximation
 *
 *     Ku = 4 d / (pi * sqrt(a^2 - eps^2))
 *
 * where d is the relay amplitude and eps the hysteresis. The oscillation needs
 * a few periods to settle, so cycles are only accumulated once consecutive
 * periods and amplitudes agree within `tolerance`. When the experiment
 * completes (or times out) the output returns to `output_bias`.
 *
 * update() is noexcept, allocation-free, and O(1).
 */
template <std::floating_point T> class RelayAutotuner {
  public:
    explicit constexpr RelayAutotuner(RelayAutotuneConfig<T> config) noexcept : config_{config} {
        reset();
    }

    /**
     * Pure relay switching without timing analysis (no tuning result).
     */
    [[nodiscard]] constexpr T update(T setpoint, T measurement) noexcept {
        switch_relay(setpoint - measurement);
        state_.output = relay_output();
        return state_.output;
    }

    /**
     * Relay switching plus limit-cycle analysis. Call once per control period.
     */
    [[nodiscard]] T update(T setpoint, T measurement, T dt) noexcept {
        if (result_.complete || result_.timed_out) {
            state_.output = config_.output_bias;
            return state_.output;
        }
        if (!is_finite(dt) || dt <= T{0} || !is_finite(setpoint) || !is_finite(measurement)) {
            return state_.output;
        }

        elapsed_ += dt;
        if (config_.timeout > T{0} && elapsed_ > config_.timeout) {
            result_.timed_out = true;
            state_.output = config_.output_bias;
            return state_.output;
        }

        if (measurement > cycle_max_) {
            cycle_max_ = measurement;
        }
        if (measurement < cycle_min_) {
            cycle_min_ = measurement;
        }

        const bool was_high = state_.high;
        switch_relay(setpoint - measurement);
        if (!was_high && state_.high) {
            on_rising_switch();
        }

        state_.output = result_.complete ? config_.output_bias : relay_output();
        return state_.output;
    }

    constexpr void reset() noexcept {
        state_.high = true;
        state_.output = relay_output();
        result_ = {};
        elapsed_ = T{0};
        last_rising_switch_ = T{0};
        rising_switches_ = 0U;
        last_period_ = T{0};
        last_amplitude_ = T{0};
        period_sum_ = T{0};
        amplitude_sum_ = T{0};
        reset_cycle_extremes();
    }

    [[nodiscard]] constexpr bool complete() const noexcept {
        return result_.complete;
    }

    [[nodiscard]] constexpr const RelayAutotuneResult<T>& result() const noexcept {
        return result_;
    }

    /// Gains for the requested rule; all zero while the experiment is incomplete.
    [[nodiscard]] constexpr PidGains<T> gains(TuningRule rule) const noexcept {
        if (!result_.complete) {
            return {};
        }
        return compute_tuning(result_.ultimate_gain, result_.ultimate_period, rule);
    }

    [[nodiscard]] constexpr const RelayAutotuneState<T>& state() const noexcept {
        return state_;
    }

    [[nodiscard]] constexpr const RelayAutotuneConfig<T>& config() const noexcept {
        return config_;
    }

  private:
    [[nodiscard]] constexpr T relay_output() const noexcept {
        return config_.output_bias +
               (state_.high ? config_.relay_amplitude : -config_.relay_amplitude);
    }

    constexpr void switch_relay(T error) noexcept {
        if (state_.high && error < -config_.hysteresis) {
            state_.high = false;
        } else if (!state_.high && error > config_.hysteresis) {
            state_.high = true;
        }
    }

    constexpr void reset_cycle_extremes() noexcept {
        cycle_max_ = -std::numeric_limits<T>::max();
        cycle_min_ = std::numeric_limits<T>::max();
    }

    [[nodiscard]] bool agrees(T value, T reference) const noexcept {
        return std::abs(value - reference) <= config_.tolerance * std::abs(reference);
    }

    void on_rising_switch() noexcept {
        ++rising_switches_;
        // Switch 1 closes the initial transient; from switch 2 on, every
        // interval between rising switches is one full limit-cycle period.
        if (rising_switches_ >= 2U) {
            const T period = elapsed_ - last_rising_switch_;
            const T amplitude = T{0.5} * (cycle_max_ - cycle_min_);
            if (rising_switches_ >= 3U && agrees(period, last_period_) &&
                agrees(amplitude, last_amplitude_)) {
                if (result_.measured_cycles == 0U) {
                    // Count the previous cycle too: it matched this one.
                    period_sum_ = last_period_;
                    amplitude_sum_ = last_amplitude_;
                    result_.measured_cycles = 1U;
                }
                period_sum_ += period;
                amplitude_sum_ += amplitude;
                ++result_.measured_cycles;
            } else {
                // Still converging: restart the average.
                result_.measured_cycles = 0U;
                period_sum_ = T{0};
                amplitude_sum_ = T{0};
            }
            last_period_ = period;
            last_amplitude_ = amplitude;
            if (result_.measured_cycles >= config_.cycles) {
                finish();
            }
        }
        last_rising_switch_ = elapsed_;
        reset_cycle_extremes();
    }

    void finish() noexcept {
        const T cycles = static_cast<T>(result_.measured_cycles);
        const T amplitude = amplitude_sum_ / cycles;
        const T hysteresis_sq = config_.hysteresis * config_.hysteresis;
        const T effective = amplitude * amplitude > hysteresis_sq
                                ? std::sqrt((amplitude * amplitude) - hysteresis_sq)
                                : amplitude;
        result_.oscillation_amplitude = amplitude;
        result_.ultimate_period = period_sum_ / cycles;
        result_.ultimate_gain = effective > T{0} ? (T{4} * config_.relay_amplitude) /
                                                       (std::numbers::pi_v<T> * effective)
                                                 : T{0};
        result_.complete = result_.ultimate_gain > T{0} && result_.ultimate_period > T{0};
        if (!result_.complete) {
            result_.timed_out = true;
        }
    }

    RelayAutotuneConfig<T> config_{};
    RelayAutotuneState<T> state_{};
    RelayAutotuneResult<T> result_{};
    T elapsed_{0};
    T last_rising_switch_{0};
    T last_period_{0};
    T last_amplitude_{0};
    T period_sum_{0};
    T amplitude_sum_{0};
    T cycle_max_{0};
    T cycle_min_{0};
    std::uint32_t rising_switches_{0};
};

} // namespace pidpp
