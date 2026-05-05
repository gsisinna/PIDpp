#pragma once

#include <concepts>
#include <span>

#include <pidpp/controllers/pid_config.hpp>

namespace pidpp::experimental {

template <std::floating_point T> struct GainSchedulePoint {
    T scheduling_value{0};
    PidConfig<T> config{};
};

template <std::floating_point T> struct GainScheduledPidControllerConfig {
    std::span<const GainSchedulePoint<T>> points{};
};

} // namespace pidpp::experimental
