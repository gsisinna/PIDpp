#pragma once

#include <pidpp/autotune/relay_autotuner.hpp>
#include <pidpp/controllers/anti_windup.hpp>
#include <pidpp/controllers/derivative_mode.hpp>
#include <pidpp/controllers/feedforward.hpp>
#include <pidpp/controllers/pid_config.hpp>
#include <pidpp/controllers/pid_controller.hpp>
#include <pidpp/core/numeric.hpp>
#include <pidpp/core/result.hpp>
#include <pidpp/core/saturation.hpp>
#include <pidpp/core/time.hpp>
#include <pidpp/filters/low_pass_filter.hpp>
#include <pidpp/filters/moving_average.hpp>
#include <pidpp/metrics/step_response.hpp>
#include <pidpp/simulation/dc_motor.hpp>
#include <pidpp/simulation/first_order_plant.hpp>
#include <pidpp/simulation/mass_spring_damper.hpp>
#include <pidpp/version.hpp>

namespace pidpp {

[[nodiscard]] const char* version() noexcept;

} // namespace pidpp
