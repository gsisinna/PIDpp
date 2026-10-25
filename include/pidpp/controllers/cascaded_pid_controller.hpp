#pragma once

#include <concepts>
#include <cstdint>

#include <pidpp/controllers/pid_controller.hpp>

namespace pidpp {

/**
 * Configuration of a two-stage cascade.
 *
 * The outer loop (e.g. position) produces the setpoint of the inner loop
 * (e.g. velocity), so the outer output limits act as the inner setpoint
 * limits (maximum velocity). The inner loop produces the actuator command.
 */
template <std::floating_point T> struct CascadedPidConfig {
    PidConfig<T> outer{};
    PidConfig<T> inner{};
    /// Run the outer loop once every `outer_decimation` inner updates (>= 1).
    std::uint32_t outer_decimation{1};

    [[nodiscard]] Result validate() const noexcept {
        if (const auto result = outer.validate(); !result) {
            return result;
        }
        if (const auto result = inner.validate(); !result) {
            return result;
        }
        if (outer_decimation == 0U) {
            return Result::error(ErrorCode::InvalidConfiguration,
                                 "outer decimation must be at least 1");
        }
        return Result::ok();
    }
};

/**
 * Cascaded (nested) PID controller, the standard servo structure in robotics:
 * position -> velocity -> torque/voltage, or attitude -> rate -> motor mix.
 *
 * Reference velocity/acceleration feedforward inputs are forwarded to both
 * stages; each stage applies them according to its own FeedforwardConfig.
 * A common setup is `outer.feedforward = {.velocity_gain = 1, .enabled = true}`
 * so the profile velocity bypasses the position loop, plus an inner
 * acceleration gain equal to the reflected inertia.
 *
 * Like PidController, update() is noexcept, allocation-free, and O(1).
 */
template <std::floating_point T> class CascadedPidController {
  public:
    explicit constexpr CascadedPidController(const CascadedPidConfig<T>& config) noexcept
        : outer_{config.outer}, inner_{config.inner},
          outer_decimation_{config.outer_decimation == 0U ? 1U : config.outer_decimation} {}

    [[nodiscard]] T update(T outer_setpoint, T outer_measurement, T inner_measurement,
                           T dt) noexcept {
        return update(outer_setpoint, outer_measurement, inner_measurement, dt, T{0}, T{0});
    }

    /**
     * @param dt inner loop period; the outer loop integrates over
     *           `dt * outer_decimation`.
     */
    [[nodiscard]] T update(T outer_setpoint, T outer_measurement, T inner_measurement, T dt,
                           T feedforward_velocity, T feedforward_acceleration) noexcept {
        if (tick_ == 0U) {
            inner_setpoint_ = outer_.update(outer_setpoint, outer_measurement,
                                            dt * static_cast<T>(outer_decimation_),
                                            feedforward_velocity, feedforward_acceleration);
        }
        tick_ = (tick_ + 1U) % outer_decimation_;
        return inner_.update(inner_setpoint_, inner_measurement, dt, feedforward_velocity,
                             feedforward_acceleration);
    }

    constexpr void reset() noexcept {
        outer_.reset();
        inner_.reset();
        inner_setpoint_ = T{0};
        tick_ = 0U;
    }

    /// Setpoint currently commanded to the inner loop (outer loop output).
    [[nodiscard]] constexpr T inner_setpoint() const noexcept {
        return inner_setpoint_;
    }

    [[nodiscard]] constexpr PidController<T>& outer() noexcept {
        return outer_;
    }

    [[nodiscard]] constexpr const PidController<T>& outer() const noexcept {
        return outer_;
    }

    [[nodiscard]] constexpr PidController<T>& inner() noexcept {
        return inner_;
    }

    [[nodiscard]] constexpr const PidController<T>& inner() const noexcept {
        return inner_;
    }

  private:
    PidController<T> outer_;
    PidController<T> inner_;
    std::uint32_t outer_decimation_{1};
    std::uint32_t tick_{0};
    T inner_setpoint_{0};
};

} // namespace pidpp
