#pragma once

#include <concepts>

namespace pidpp {

/**
 * Simple feedforward terms that can be added to a PID command.
 */
template <std::floating_point T> struct FeedforwardConfig {
    T velocity_gain{0};
    T acceleration_gain{0};
    T static_gain{0};
    bool enabled{false};
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
        return (config_.velocity_gain * velocity) + (config_.acceleration_gain * acceleration) +
               config_.static_gain;
    }

  private:
    FeedforwardConfig<T> config_{};
};

} // namespace pidpp
