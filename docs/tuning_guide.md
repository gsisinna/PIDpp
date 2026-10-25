# Tuning Guide

A practical procedure for tuning PID++ loops on robots and other mechatronic systems. The
[PID++ Control Lab](../README.md#pid-control-lab) and the [examples](examples.md) let you try
every step in simulation first.

## 0. Before touching gains

1. **Get the units and signs right.** Positive command must move the measurement up. If it does
   not, flip the sign of the actuator or of the measurement, never the gains (PID++ gains are
   non-negative by design).
2. **Set the output limits** to what the actuator can really deliver (`output_min/max`: volts,
   amps, N*m, N). Saturation is where most PID problems start, and anti-windup only works if
   the controller knows the limits.
3. **Pick the loop rate.** Run the loop 10 to 20 times faster than the closed-loop bandwidth you
   want. Pass the real measured `dt` to `update()`.
4. **Add feedforward for everything you can model.** It is cheaper than gain:
   - constant load (gravity on an elevator, hover thrust): `feedforward.static_gain`;
   - angle-dependent load (arm gravity): `update_with_feedforward(..., m*g*l*cos(q))`;
   - motion along a profile: `velocity_gain` (kV) and `acceleration_gain` (kA);
   - static friction: `friction_gain` (kS).

## 1. Proportional

Start with `ki = kd = 0`. Increase `kp` until the response is fast enough, or until it starts to
oscillate, then back off to about half of that value. P-only control usually leaves a
steady-state error against constant loads.

## 2. Derivative

Add `kd` to damp overshoot and allow a higher `kp`.

- Use `DerivativeMode::OnMeasurement` when the setpoint changes in steps (operator commands,
  waypoints): it avoids the derivative kick.
- Use `DerivativeMode::OnError` when the setpoint is a smooth trajectory: damping relative to
  the moving reference tracks better (see `arm_gravity_compensation`).
- Always set `derivative_filter_tau` on real sensors. Start with about one tenth of the
  closed-loop time constant, or 2 to 10 sample periods. Encoders, ADCs, and IMUs all produce
  noise that the derivative amplifies (see `sensor_noise_filtering`).

## 3. Integral

Add `ki` last, only to remove steady-state error. A good starting point is
`ki = kp / (5 * settling_time)`. Then:

- set `integral_min/max`: they bound the accumulated error (error * seconds), so the maximum
  integral contribution is `ki * integral_max`;
- pick an anti-windup strategy (below).

## Anti-windup strategies

| Strategy | Behavior | Use when |
| --- | --- | --- |
| `None` | Integrates freely within `integral_min/max` | Analysis, or loops that never saturate |
| `Clamping` | Dynamic clamping: the integral may not push the output further past the limits than P, D, and FF already do | Good default |
| `ConditionalIntegration` | Freezes the integrator while saturated and the error would deepen saturation | Large setpoint steps, actuators that saturate often |
| `BackCalculation` | Bleeds the integrator by `back_calculation_gain * (saturated - unsaturated) * dt` | Smooth recovery from saturation; tune the tracking gain |

## Common robotics recipes

| Loop | Typical structure |
| --- | --- |
| Motor current/torque | PI, high rate, `Clamping` |
| Motor or wheel speed | PI + kV feedforward, `ConditionalIntegration` |
| Joint position (no gravity) | PD or PID, derivative on measurement, filtered derivative |
| Joint position (gravity) | PID + gravity feedforward via `update_with_feedforward` |
| Servo / robot joint along trajectories | `CascadedPidController` + profile feedforward |
| Mobile robot heading | PD/PID with `continuous_input = true`, yaw-rate limits |
| Drone altitude | PID + hover feedforward, `output_rate_limit`, slewed setpoint |
| Varying payload / speed | `GainScheduledPidController` |

## Cascades

Tune the inner loop first with the outer loop open, then the outer loop. The inner loop should be
3 to 5 times faster than the outer loop. Set the outer output limits to the inner setpoint
limits you want (for example maximum velocity).

## Autotuning

When no model is available, use `RelayAutotuner`:

1. Bring the plant to its operating point and set `output_bias` to the command that holds it.
2. Choose `relay_amplitude` large enough for a clear oscillation but safe for the mechanism.
3. Set `hysteresis` above the measurement noise.
4. Run until `complete()`, then compute gains with `gains(rule)`.

Start with `TyreusLuybenPi` or `NoOvershootPid` for mechanical systems; Ziegler-Nichols is
aggressive. The relay method needs dead time or at least three lags in the loop; a pure
first-order system does not sustain a meaningful limit cycle.

## Other tools

- `error_deadband`: ignore errors smaller than the sensor resolution or mechanical backlash, to
  stop hunting around the target. The deadband is continuous (no output step at its edge).
- `output_rate_limit`: limit how fast the command may change (N*m/s, V/s) to protect gearboxes
  and power electronics.
- `bumpless_transfer()`: switch from manual or another controller to this PID without an
  output jump.
- `set_gains()` / `set_config()` rescale the integrator so runtime gain changes do not bump the
  output.

Validate every `PidConfig<T>` before putting a controller into service.
