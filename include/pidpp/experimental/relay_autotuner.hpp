#pragma once

#include <concepts>

#include <pidpp/autotune/relay_autotuner.hpp>

namespace pidpp::experimental {

/// Deprecated alias; use pidpp::RelayAutotuner.
template <std::floating_point T> using RelayAutoTuner = pidpp::RelayAutotuner<T>;

} // namespace pidpp::experimental
