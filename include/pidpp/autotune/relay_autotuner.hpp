#pragma once

#include <concepts>

#include <pidpp/controllers/pid_config.hpp>
#include <pidpp/core/numeric.hpp>
#include <pidpp/core/result.hpp>

namespace pidpp {

/**
 * Configuration for the experimental relay autotuning helper.
 */
template <std::floating_point T>
struct RelayAutotuneConfig {
    T relay_amplitude{1};
    T hysteresis{0};
    T output_bias{0};

    [[nodiscard]] Result validate() const noexcept
    {
        if (!is_finite(relay_amplitude) || relay_amplitude <= T{0}) {
            return Result::error(ErrorCode::InvalidConfiguration, "relay amplitude must be positive");
        }
        if (!is_finite(hysteresis) || hysteresis < T{0}) {
            return Result::error(ErrorCode::InvalidConfiguration, "hysteresis must be non-negative");
        }
        if (!is_finite(output_bias)) {
            return Result::error(ErrorCode::InvalidConfiguration, "output bias must be finite");
        }
        return Result::ok();
    }
};

template <std::floating_point T>
struct RelayAutotuneState {
    T output{0};
    bool high{true};
};

template <std::floating_point T>
class RelayAutotuner {
public:
    explicit constexpr RelayAutotuner(RelayAutotuneConfig<T> config) noexcept
        : config_{config}
    {
        state_.output = config_.output_bias + config_.relay_amplitude;
    }

    [[nodiscard]] constexpr T update(T setpoint, T measurement) noexcept
    {
        const T error = setpoint - measurement;
        if (state_.high && error < -config_.hysteresis) {
            state_.high = false;
        } else if (!state_.high && error > config_.hysteresis) {
            state_.high = true;
        }

        state_.output = config_.output_bias +
                        (state_.high ? config_.relay_amplitude : -config_.relay_amplitude);
        return state_.output;
    }

    constexpr void reset() noexcept
    {
        state_.high = true;
        state_.output = config_.output_bias + config_.relay_amplitude;
    }

    [[nodiscard]] constexpr const RelayAutotuneState<T>& state() const noexcept
    {
        return state_;
    }

private:
    RelayAutotuneConfig<T> config_{};
    RelayAutotuneState<T> state_{};
};

} // namespace pidpp
