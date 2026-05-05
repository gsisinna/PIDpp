#pragma once

namespace pidpp {

/**
 * Selects the signal used for derivative action.
 */
enum class DerivativeMode { OnError, OnMeasurement };

} // namespace pidpp
