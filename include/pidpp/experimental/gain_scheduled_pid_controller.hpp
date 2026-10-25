#pragma once

#include <concepts>
#include <cstddef>

#include <pidpp/controllers/gain_scheduled_pid_controller.hpp>

namespace pidpp::experimental {

/**
 * Deprecated experimental aliases kept for source compatibility.
 *
 * Gain scheduling graduated to the stable API in 0.2.0: use
 * pidpp::GainSchedulePoint and pidpp::GainScheduledPidController.
 */
template <std::floating_point T> using GainSchedulePoint = pidpp::GainSchedulePoint<T>;

template <std::floating_point T, std::size_t MaxPoints>
using GainScheduledPidController = pidpp::GainScheduledPidController<T, MaxPoints>;

} // namespace pidpp::experimental
