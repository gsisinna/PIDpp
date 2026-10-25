#pragma once

#include <concepts>

#include <pidpp/controllers/cascaded_pid_controller.hpp>

namespace pidpp::experimental {

/**
 * Deprecated experimental aliases kept for source compatibility.
 *
 * Cascaded PID graduated to the stable API in 0.2.0: use
 * pidpp::CascadedPidConfig and pidpp::CascadedPidController.
 */
template <std::floating_point T> using CascadedPidControllerConfig = pidpp::CascadedPidConfig<T>;

template <std::floating_point T> using CascadedPidController = pidpp::CascadedPidController<T>;

} // namespace pidpp::experimental
