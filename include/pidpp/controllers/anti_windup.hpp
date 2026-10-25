#pragma once

namespace pidpp {

/**
 * Anti-windup policy used by PidController when output saturation occurs.
 *
 * The static integral bounds (`PidConfig::integral_min/max`) are always
 * enforced as a hard safety limit, whatever strategy is selected.
 *
 * - None: integrate normally; only the static integral bounds apply.
 * - Clamping: dynamic integrator clamping. The integral contribution is
 *   limited so that it never pushes the output further past the output
 *   limits than the P, D, and feedforward terms already do.
 * - ConditionalIntegration: freeze the integrator while the output is
 *   saturated and the error would drive it deeper into saturation.
 * - BackCalculation: bleed the integrator with
 *   `back_calculation_gain * (saturated - unsaturated) * dt`.
 */
enum class AntiWindupStrategy { None, Clamping, ConditionalIntegration, BackCalculation };

} // namespace pidpp
