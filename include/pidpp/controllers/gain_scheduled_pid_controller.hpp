#pragma once

#include <array>
#include <concepts>
#include <cstddef>
#include <span>

#include <pidpp/controllers/pid_controller.hpp>

namespace pidpp {

/**
 * One breakpoint of a gain schedule: the gains to use when the scheduling
 * variable equals `scheduling_value`.
 */
template <std::floating_point T> struct GainSchedulePoint {
    T scheduling_value{0};
    PidGains<T> gains{};
};

/**
 * PID controller whose gains are interpolated from a fixed-capacity table.
 *
 * Classic robotics uses: gains as a function of payload mass, arm extension,
 * vehicle speed, battery voltage, or operating point of a nonlinear plant.
 * Between breakpoints the gains are linearly interpolated; outside the table
 * the nearest breakpoint is held. Gain changes are bumpless (see
 * PidController::set_gains()).
 *
 * Storage is an embedded std::array of `MaxPoints` entries, so the controller
 * never allocates. set_schedule() validates the table and runs outside the
 * real-time path; update() is noexcept and O(MaxPoints).
 */
template <std::floating_point T, std::size_t MaxPoints> class GainScheduledPidController {
    static_assert(MaxPoints > 0, "MaxPoints must be greater than zero");

  public:
    explicit constexpr GainScheduledPidController(const PidConfig<T>& base_config) noexcept
        : controller_{base_config} {}

    /**
     * Copies the schedule into internal storage. Points must be sorted by
     * strictly increasing scheduling value and contain valid gains.
     */
    [[nodiscard]] Result set_schedule(std::span<const GainSchedulePoint<T>> points) noexcept {
        if (points.empty() || points.size() > MaxPoints) {
            return Result::error(ErrorCode::InvalidConfiguration,
                                 "gain schedule size must be in [1, MaxPoints]");
        }
        for (std::size_t i = 0; i < points.size(); ++i) {
            if (!is_finite(points[i].scheduling_value)) {
                return Result::error(ErrorCode::InvalidConfiguration,
                                     "scheduling values must be finite");
            }
            if (const auto result = points[i].gains.validate(); !result) {
                return result;
            }
            if (i > 0 && points[i].scheduling_value <= points[i - 1].scheduling_value) {
                return Result::error(ErrorCode::InvalidConfiguration,
                                     "scheduling values must be strictly increasing");
            }
        }
        for (std::size_t i = 0; i < points.size(); ++i) {
            points_[i] = points[i];
        }
        size_ = points.size();
        controller_.set_gains(points_[0].gains);
        return Result::ok();
    }

    /// Interpolated gains for a given scheduling value.
    [[nodiscard]] constexpr PidGains<T> gains_at(T scheduling_value) const noexcept {
        if (size_ == 0U) {
            return controller_.config().gains();
        }
        if (!(scheduling_value > points_[0].scheduling_value)) {
            return points_[0].gains;
        }
        for (std::size_t i = 1; i < size_; ++i) {
            if (scheduling_value <= points_[i].scheduling_value) {
                const auto& lo = points_[i - 1];
                const auto& hi = points_[i];
                const T ratio = (scheduling_value - lo.scheduling_value) /
                                (hi.scheduling_value - lo.scheduling_value);
                return {lerp(lo.gains.kp, hi.gains.kp, ratio),
                        lerp(lo.gains.ki, hi.gains.ki, ratio),
                        lerp(lo.gains.kd, hi.gains.kd, ratio)};
            }
        }
        return points_[size_ - 1U].gains;
    }

    [[nodiscard]] T update(T scheduling_value, T setpoint, T measurement, T dt) noexcept {
        return update(scheduling_value, setpoint, measurement, dt, T{0}, T{0});
    }

    [[nodiscard]] T update(T scheduling_value, T setpoint, T measurement, T dt,
                           T feedforward_velocity, T feedforward_acceleration) noexcept {
        if (is_finite(scheduling_value)) {
            controller_.set_gains(gains_at(scheduling_value));
        }
        return controller_.update(setpoint, measurement, dt, feedforward_velocity,
                                  feedforward_acceleration);
    }

    constexpr void reset() noexcept {
        controller_.reset();
    }

    [[nodiscard]] constexpr std::size_t size() const noexcept {
        return size_;
    }

    [[nodiscard]] constexpr PidController<T>& controller() noexcept {
        return controller_;
    }

    [[nodiscard]] constexpr const PidController<T>& controller() const noexcept {
        return controller_;
    }

  private:
    [[nodiscard]] static constexpr T lerp(T a, T b, T ratio) noexcept {
        return a + ((b - a) * ratio);
    }

    PidController<T> controller_;
    std::array<GainSchedulePoint<T>, MaxPoints> points_{};
    std::size_t size_{0};
};

} // namespace pidpp
