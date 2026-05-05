#pragma once

#include <pidpp/autotune/relay_autotuner.hpp>

namespace pidpp::experimental {

template <std::floating_point T>
using RelayAutoTuner = pidpp::RelayAutotuner<T>;

} // namespace pidpp::experimental
