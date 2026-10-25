#pragma once

/**
 * PID++ facade header: includes the whole stable public API.
 *
 * Experimental headers under <pidpp/experimental/...> are opt-in and are
 * intentionally not included here.
 */

#include <pidpp/autotune/relay_autotuner.hpp>
#include <pidpp/controllers/anti_windup.hpp>
#include <pidpp/controllers/cascaded_pid_controller.hpp>
#include <pidpp/controllers/derivative_mode.hpp>
#include <pidpp/controllers/feedforward.hpp>
#include <pidpp/controllers/gain_scheduled_pid_controller.hpp>
#include <pidpp/controllers/pid_config.hpp>
#include <pidpp/controllers/pid_controller.hpp>
#include <pidpp/core/angles.hpp>
#include <pidpp/core/numeric.hpp>
#include <pidpp/core/result.hpp>
#include <pidpp/core/saturation.hpp>
#include <pidpp/core/time.hpp>
#include <pidpp/filters/low_pass_filter.hpp>
#include <pidpp/filters/moving_average.hpp>
#include <pidpp/filters/slew_rate_limiter.hpp>
#include <pidpp/metrics/step_response.hpp>
#include <pidpp/motion/trapezoidal_profile.hpp>
#include <pidpp/simulation/dc_motor.hpp>
#include <pidpp/simulation/differential_drive.hpp>
#include <pidpp/simulation/first_order_plant.hpp>
#include <pidpp/simulation/mass_spring_damper.hpp>
#include <pidpp/simulation/single_link_arm.hpp>
#include <pidpp/simulation/vertical_thrust_plant.hpp>
#include <pidpp/version.hpp>

namespace pidpp {

/// Library version string from the compiled library (matches version_string()).
[[nodiscard]] const char* version() noexcept;

} // namespace pidpp
