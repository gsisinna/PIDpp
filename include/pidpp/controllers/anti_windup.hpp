#pragma once

namespace pidpp {

/**
 * Anti-windup policy used by PidController when output saturation occurs.
 */
enum class AntiWindupStrategy { None, Clamping, ConditionalIntegration, BackCalculation };

} // namespace pidpp
