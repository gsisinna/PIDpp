#pragma once

#include <algorithm>
#include <concepts>

#include <pidpp/controllers/pid_config.hpp>
#include <pidpp/core/angles.hpp>
#include <pidpp/core/saturation.hpp>
#include <pidpp/core/time.hpp>
#include <pidpp/filters/low_pass_filter.hpp>

namespace pidpp {

/**
 * Observable controller state. All terms are those of the most recent valid
 * update and are expressed in output units unless noted otherwise.
 */
template <std::floating_point T> struct PidState {
    /// Error after continuous-input wrapping and deadband.
    T error{0};
    T previous_error{0};
    T previous_measurement{0};
    /// Accumulated error in error * seconds.
    T integral{0};
    /// Derivative signal (after optional filtering), in error units per second.
    T derivative{0};
    T proportional_term{0};
    T integral_term{0};
    T derivative_term{0};
    T feedforward_term{0};
    T unsaturated_output{0};
    T output{0};
    /// True when the output was clipped by output limits or the rate limit.
    bool saturated{false};
    bool initialized{false};
    /// False when the last call was rejected because dt or an input was invalid.
    bool last_dt_valid{true};
};

/**
 * Allocation-free PID controller for real-time control loops.
 *
 * update() is noexcept, performs deterministic O(1) work, and stores all state
 * inside this object. Validate PidConfig before constructing controllers used
 * in safety-critical loops.
 */
template <std::floating_point T> class PidController {
  public:
    explicit constexpr PidController(const PidConfig<T>& config) noexcept
        : config_{config}, derivative_filter_{config.derivative_filter_tau},
          feedforward_{config.feedforward} {}

    [[nodiscard]] T update(T setpoint, T measurement, T dt) noexcept {
        return update(setpoint, measurement, dt, T{0}, T{0});
    }

    /**
     * Computes one control step.
     *
     * @param setpoint                 desired value of the controlled variable
     * @param measurement              measured value of the controlled variable
     * @param dt                       time since the previous update in seconds (> 0)
     * @param feedforward_velocity     reference velocity passed to the feedforward model
     * @param feedforward_acceleration reference acceleration passed to the feedforward model
     * @return the saturated (and rate limited) actuator command
     */
    [[nodiscard]] T update(T setpoint, T measurement, T dt, T feedforward_velocity,
                           T feedforward_acceleration) noexcept {
        if (!is_finite(feedforward_velocity) || !is_finite(feedforward_acceleration)) {
            state_.last_dt_valid = false;
            return state_.output;
        }
        return step(setpoint, measurement, dt,
                    feedforward_.calculate(feedforward_velocity, feedforward_acceleration));
    }

    /**
     * Computes one control step with an externally computed feedforward term.
     *
     * `feedforward` is added in output units before saturation, bypassing
     * FeedforwardConfig. Use it for model-based terms such as gravity
     * compensation `m*g*l*cos(q)` or inverse-dynamics torques. Anti-windup
     * accounts for it like any other term.
     */
    [[nodiscard]] T update_with_feedforward(T setpoint, T measurement, T dt,
                                            T feedforward) noexcept {
        return step(setpoint, measurement, dt, feedforward);
    }

    /// Clears all controller state, including the integrator and output memory.
    constexpr void reset() noexcept {
        state_ = {};
        derivative_filter_.reset();
    }

    constexpr void reset_integral(T integral = T{0}) noexcept {
        state_.integral = clamp(integral, config_.integral_min, config_.integral_max);
    }

    /**
     * Bumpless transfer from manual (or another controller) to this PID.
     *
     * Resets the controller and preloads the integrator so that the next
     * update() at the given setpoint/measurement reproduces `current_output`
     * (reference velocity/acceleration feedforward assumed zero). Use it when
     * engaging closed-loop control on an actuator that is already moving.
     */
    void bumpless_transfer(T current_output, T setpoint, T measurement) noexcept {
        reset();
        const T output = clamp(current_output, config_.output_min, config_.output_max);
        state_.output = output;
        if (config_.ki > T{0} && is_finite(setpoint) && is_finite(measurement)) {
            const T error = compute_error(setpoint, measurement);
            const T integral_term =
                output - (config_.kp * error) - feedforward_.calculate(T{0}, T{0});
            reset_integral(integral_term / config_.ki);
        }
        state_.integral_term = config_.ki * state_.integral;
    }

    /**
     * Replaces the configuration at runtime.
     *
     * The integrator is rescaled when `ki` changes so the integral
     * contribution to the output stays continuous (bumpless gain change).
     */
    constexpr void set_config(const PidConfig<T>& config) noexcept {
        const T previous_ki = config_.ki;
        config_ = config;
        derivative_filter_.set_tau(config.derivative_filter_tau);
        feedforward_.set_config(config.feedforward);
        rescale_integral(previous_ki);
    }

    /// Updates gains at runtime with a bumpless integral rescale.
    constexpr void set_gains(T kp, T ki, T kd) noexcept {
        const T previous_ki = config_.ki;
        config_.kp = kp;
        config_.ki = ki;
        config_.kd = kd;
        rescale_integral(previous_ki);
    }

    constexpr void set_gains(const PidGains<T>& gains) noexcept {
        set_gains(gains.kp, gains.ki, gains.kd);
    }

    [[nodiscard]] constexpr const PidConfig<T>& config() const noexcept {
        return config_;
    }

    [[nodiscard]] constexpr const PidState<T>& state() const noexcept {
        return state_;
    }

  private:
    struct Bounds {
        T lower;
        T upper;
    };

    [[nodiscard]] T step(T setpoint, T measurement, T dt, T feedforward) noexcept {
        // Real-time contract: invalid inputs preserve the previous command
        // without throwing, logging, or allocating.
        if (!validate_time_step(dt) || !is_finite(setpoint) || !is_finite(measurement) ||
            !is_finite(feedforward)) {
            state_.last_dt_valid = false;
            return state_.output;
        }

        state_.last_dt_valid = true;
        const T error = compute_error(setpoint, measurement);

        if (!state_.initialized) {
            state_.previous_error = error;
            state_.previous_measurement = measurement;
            derivative_filter_.reset(T{0});
            state_.initialized = true;
        }

        state_.error = error;
        state_.proportional_term = config_.kp * error;
        state_.derivative = calculate_derivative(error, measurement, dt);
        state_.derivative_term = config_.kd * state_.derivative;
        state_.feedforward_term = feedforward;

        const Bounds bounds = output_bounds(dt);

        const T candidate_integral =
            clamp(state_.integral + (error * dt), config_.integral_min, config_.integral_max);
        const T candidate_output = state_.proportional_term + (config_.ki * candidate_integral) +
                                   state_.derivative_term + state_.feedforward_term;
        const T saturated_candidate = clamp(candidate_output, bounds.lower, bounds.upper);

        apply_anti_windup(error, dt, candidate_integral, candidate_output, saturated_candidate,
                          bounds);

        state_.integral_term = config_.ki * state_.integral;
        state_.unsaturated_output = state_.proportional_term + state_.integral_term +
                                    state_.derivative_term + state_.feedforward_term;
        state_.output = clamp(state_.unsaturated_output, bounds.lower, bounds.upper);
        state_.saturated = state_.output != state_.unsaturated_output;
        state_.previous_error = error;
        state_.previous_measurement = measurement;
        return state_.output;
    }

    [[nodiscard]] T wrap_delta(T delta) const noexcept {
        if (!config_.continuous_input) {
            return delta;
        }
        const T half_span = (config_.input_range_max - config_.input_range_min) / T{2};
        return wrap_to_range(delta, -half_span, half_span);
    }

    [[nodiscard]] T compute_error(T setpoint, T measurement) const noexcept {
        return apply_deadband(wrap_delta(setpoint - measurement), config_.error_deadband);
    }

    [[nodiscard]] Bounds output_bounds(T dt) const noexcept {
        Bounds bounds{config_.output_min, config_.output_max};
        if (config_.output_rate_limit > T{0}) {
            const T max_step = config_.output_rate_limit * dt;
            bounds.lower = std::max(bounds.lower, state_.output - max_step);
            bounds.upper = std::min(bounds.upper, state_.output + max_step);
            if (bounds.lower > bounds.upper) {
                // The previous output lies outside the (possibly reconfigured)
                // output limits: the hard limits win over the slew rate.
                const T limited = clamp(state_.output, config_.output_min, config_.output_max);
                bounds = {limited, limited};
            }
        }
        return bounds;
    }

    constexpr void rescale_integral(T previous_ki) noexcept {
        if (previous_ki > T{0} && config_.ki > T{0} && previous_ki != config_.ki) {
            state_.integral *= previous_ki / config_.ki;
        }
        state_.integral = clamp(state_.integral, config_.integral_min, config_.integral_max);
        state_.integral_term = config_.ki * state_.integral;
    }

    [[nodiscard]] T calculate_derivative(T error, T measurement, T dt) noexcept {
        const T raw_derivative = config_.derivative_mode == DerivativeMode::OnError
                                     ? (error - state_.previous_error) / dt
                                     : -wrap_delta(measurement - state_.previous_measurement) / dt;

        if (config_.derivative_filter_tau > T{0}) {
            return derivative_filter_.update(raw_derivative, dt);
        }
        return raw_derivative;
    }

    void apply_anti_windup(T error, T dt, T candidate_integral, T candidate_output,
                           T saturated_candidate, Bounds bounds) noexcept {
        switch (config_.anti_windup) {
        case AntiWindupStrategy::None:
            state_.integral = candidate_integral;
            break;
        case AntiWindupStrategy::Clamping: {
            T integral = candidate_integral;
            if (config_.ki > T{0}) {
                const T other_terms =
                    state_.proportional_term + state_.derivative_term + state_.feedforward_term;
                const T max_term = std::max(bounds.upper - other_terms, T{0});
                const T min_term = std::min(bounds.lower - other_terms, T{0});
                integral = clamp(integral, min_term / config_.ki, max_term / config_.ki);
            }
            state_.integral = clamp(integral, config_.integral_min, config_.integral_max);
            break;
        }
        case AntiWindupStrategy::ConditionalIntegration:
            if (candidate_output == saturated_candidate ||
                (candidate_output > bounds.upper && error < T{0}) ||
                (candidate_output < bounds.lower && error > T{0})) {
                state_.integral = candidate_integral;
            }
            break;
        case AntiWindupStrategy::BackCalculation: {
            const T correction =
                config_.back_calculation_gain * (saturated_candidate - candidate_output) * dt;
            state_.integral =
                clamp(candidate_integral + correction, config_.integral_min, config_.integral_max);
            break;
        }
        }
    }

    PidConfig<T> config_{};
    PidState<T> state_{};
    LowPassFilter<T> derivative_filter_{};
    Feedforward<T> feedforward_{};
};

} // namespace pidpp
