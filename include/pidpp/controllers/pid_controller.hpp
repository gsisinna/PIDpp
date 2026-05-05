#pragma once

#include <concepts>

#include <pidpp/controllers/pid_config.hpp>
#include <pidpp/core/saturation.hpp>
#include <pidpp/core/time.hpp>
#include <pidpp/filters/low_pass_filter.hpp>

namespace pidpp {

template <std::floating_point T> struct PidState {
    T error{0};
    T previous_error{0};
    T previous_measurement{0};
    T integral{0};
    T derivative{0};
    T proportional_term{0};
    T integral_term{0};
    T derivative_term{0};
    T feedforward_term{0};
    T unsaturated_output{0};
    T output{0};
    bool saturated{false};
    bool initialized{false};
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

    [[nodiscard]] T update(T setpoint, T measurement, T dt, T feedforward_velocity,
                           T feedforward_acceleration) noexcept {
        // Real-time contract: invalid inputs preserve the previous command
        // without throwing, logging, or allocating.
        if (!validate_time_step(dt) || !is_finite(setpoint) || !is_finite(measurement)) {
            state_.last_dt_valid = false;
            return state_.output;
        }

        state_.last_dt_valid = true;
        const T error = setpoint - measurement;

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
        state_.feedforward_term =
            feedforward_.calculate(feedforward_velocity, feedforward_acceleration);

        const T candidate_integral =
            clamp(state_.integral + (error * dt), config_.integral_min, config_.integral_max);

        const T candidate_integral_term = config_.ki * candidate_integral;
        const T candidate_output = state_.proportional_term + candidate_integral_term +
                                   state_.derivative_term + state_.feedforward_term;
        const T saturated_candidate =
            clamp(candidate_output, config_.output_min, config_.output_max);

        apply_anti_windup(error, dt, candidate_integral, candidate_output, saturated_candidate);

        state_.integral_term = config_.ki * state_.integral;
        state_.unsaturated_output = state_.proportional_term + state_.integral_term +
                                    state_.derivative_term + state_.feedforward_term;
        state_.output = clamp(state_.unsaturated_output, config_.output_min, config_.output_max);
        state_.saturated = state_.output != state_.unsaturated_output;
        state_.previous_error = error;
        state_.previous_measurement = measurement;
        return state_.output;
    }

    constexpr void reset() noexcept {
        state_ = {};
        derivative_filter_.reset();
    }

    constexpr void reset_integral(T integral = T{0}) noexcept {
        state_.integral = clamp(integral, config_.integral_min, config_.integral_max);
    }

    constexpr void set_config(const PidConfig<T>& config) noexcept {
        config_ = config;
        derivative_filter_.set_tau(config.derivative_filter_tau);
        feedforward_.set_config(config.feedforward);
        state_.integral = clamp(state_.integral, config_.integral_min, config_.integral_max);
    }

    constexpr void set_gains(T kp, T ki, T kd) noexcept {
        config_.kp = kp;
        config_.ki = ki;
        config_.kd = kd;
    }

    [[nodiscard]] constexpr const PidConfig<T>& config() const noexcept {
        return config_;
    }

    [[nodiscard]] constexpr const PidState<T>& state() const noexcept {
        return state_;
    }

  private:
    [[nodiscard]] T calculate_derivative(T error, T measurement, T dt) noexcept {
        const T raw_derivative = config_.derivative_mode == DerivativeMode::OnError
                                     ? (error - state_.previous_error) / dt
                                     : -(measurement - state_.previous_measurement) / dt;

        if (config_.derivative_filter_tau > T{0}) {
            return derivative_filter_.update(raw_derivative, dt);
        }
        return raw_derivative;
    }

    void apply_anti_windup(T error, T dt, T candidate_integral, T candidate_output,
                           T saturated_candidate) noexcept {
        switch (config_.anti_windup) {
        case AntiWindupStrategy::None:
            state_.integral = candidate_integral;
            break;
        case AntiWindupStrategy::Clamping:
            state_.integral = clamp(candidate_integral, config_.integral_min, config_.integral_max);
            break;
        case AntiWindupStrategy::ConditionalIntegration:
            if (candidate_output == saturated_candidate ||
                (candidate_output > config_.output_max && error < T{0}) ||
                (candidate_output < config_.output_min && error > T{0})) {
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
