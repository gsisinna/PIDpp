#pragma once

#include <concepts>

#include <pidpp/controllers/pid_controller.hpp>

namespace pidpp::experimental {

/**
 * Placeholder for future cascaded PID composition.
 *
 * Experimental APIs are source-unstable and may change in any release.
 */
template <std::floating_point T> struct CascadedPidControllerConfig {
    PidConfig<T> outer{};
    PidConfig<T> inner{};
};

} // namespace pidpp::experimental
