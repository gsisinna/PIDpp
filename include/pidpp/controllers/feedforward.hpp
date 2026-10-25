#pragma once

#include <concepts>

namespace pidpp {

/**
 * Model-based feedforward terms that can be added to a PID command.
 *
 *     u_ff = velocity_gain * v + acceleration_gain * a
 *          + friction_gain * sign(v) + static_gain
 *
 * Typical robotics mapping: `velocity_gain` ~ kV (back-EMF / viscous
 * friction), `acceleration_gain` ~ kA (inertia), `friction_gain` ~ kS
 * (Coulomb friction), `static_gain` ~ kG for constant loads such as the
 * weight of an elevator or the hover thrust of a drone.
 */
template <std::floating_point T> struct FeedforwardConfig {
    T velocity_gain{0};
    T acceleration_gain{0};
    T static_gain{0};
    bool enabled{false};
    T friction_gain{0};
};

template <std::floating_point T> class Feedforward {
  public:
    constexpr Feedforward() noexcept = default;

    explicit constexpr Feedforward(const FeedforwardConfig<T>& config) noexcept : config_{config} {}

    constexpr void set_config(const FeedforwardConfig<T>& config) noexcept {
        config_ = config;
    }

    [[nodiscard]] constexpr const FeedforwardConfig<T>& config() const noexcept {
        return config_;
    }

    [[nodiscard]] constexpr T calculate(T velocity, T acceleration) const noexcept {
        if (!config_.enabled) {
            return T{0};
        }
        T friction{0};
        if (velocity > T{0}) {
            friction = config_.friction_gain;
        } else if (velocity < T{0}) {
            friction = -config_.friction_gain;
        }
        return (config_.velocity_gain * velocity) + (config_.acceleration_gain * acceleration) +
               friction + config_.static_gain;
    }

  private:
    FeedforwardConfig<T> config_{};
};

} // namespace pidpp
