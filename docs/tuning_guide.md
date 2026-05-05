# Tuning Guide

Start simple and add terms only when the plant behavior requires them.

## P

Increase `kp` until the system responds quickly enough without unacceptable oscillation. P-only control often leaves steady-state error.

## PI

Add `ki` to remove steady-state error. Use output limits and an anti-windup strategy when actuators saturate.

## PD

Add `kd` to damp motion. For robotics, `DerivativeMode::OnMeasurement` is often preferable because it avoids derivative kick on setpoint changes.

## Filtering

Use `derivative_filter_tau` when measurement noise makes the derivative term too aggressive. Larger values smooth more but add delay.

## Anti-Windup

- `None`: simplest behavior, useful for analysis.
- `Clamping`: keeps integral state within configured bounds.
- `ConditionalIntegration`: stops integrating when saturation would deepen.
- `BackCalculation`: corrects integral state based on saturation error.

Validate every `PidConfig<T>` before putting a controller into service.
