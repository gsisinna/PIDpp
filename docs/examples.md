# Robotics Examples

Every program in [`examples/`](../examples) simulates a realistic scenario with the SDK's plant
models, prints a short report, and checks acceptance criteria. A failed criterion makes the
program exit with a non-zero code, and all examples are registered in CTest
(`example.<name>`), so the examples double as an end-to-end regression suite.

Pass `--csv <file>` to any example that logs time series to export them for plotting:

```sh
./build/examples/cascaded_servo_control --csv servo.csv
python -c "import pandas as pd; pd.read_csv('servo.csv').plot(x='t', y=['ref_pos', 'pos'])"
```

Gains in the examples are derived from the plant parameters stated in each file, so they are a
reasonable starting point for hardware with similar dynamics, not universal values.

## `motor_speed_control.cpp`

**Scenario.** A brushed DC motor (wheel, conveyor, spindle) must hold 50 rad/s on a 12 V
supply. At t = 4 s a 0.05 N*m load is applied, like a wheel climbing a ramp.

**Techniques.** Velocity feedforward `kV = R*b/Kt + Ke` supplies the steady-state voltage so the
PI only corrects errors. Conditional integration freezes the integrator while the voltage
saturates during spin-up, avoiding overshoot.

**Checks.** Speed within 0.5 rad/s before the load and after recovery.

## `joint_position_control.cpp`

**Scenario.** A horizontal revolute joint (no gravity) steps 90 degrees with torque limited to
60 N*m.

**Techniques.** Derivative on measurement avoids the derivative kick on the setpoint step;
back-calculation bleeds the integrator while torque saturates. Step metrics report overshoot,
rise time, and settling time.

## `cascaded_servo_control.cpp`

**Scenario.** A servo drive moves its motor one full revolution along a trapezoidal profile.

**Techniques.** The industry-standard cascade: an outer P position loop (1 kHz) feeds a velocity
setpoint to an inner PI velocity loop (2 kHz) that outputs volts. The profile velocity bypasses
the position loop (`outer.feedforward.velocity_gain = 1`) and the inner loop adds the
voltage model `kV * v + kA * a`. `outer_decimation = 2` runs the outer loop at half rate.

**Checks.** Tracking error below 0.05 rad and final error below 1 mrad.

## `trajectory_tracking.cpp`

**Scenario.** A pick-and-place joint follows a 90 degree trapezoidal move twice with identical
PID gains: feedback only, then with feedforward.

**Lesson.** Feedback can only react to error, so it always lags a moving reference and its
integrator charges up during the move. Feedforward from the joint model (`kA` = inertia,
`kV` = viscous friction) provides the torque the move needs; in this example the RMS tracking
error drops by about three orders of magnitude.

## `arm_gravity_compensation.cpp`

**Scenario.** A shoulder joint lifts a 0.6 m link from hanging down (-90 degrees) to 30 degrees
above horizontal, then picks up a 0.5 kg object at t = 4 s.

**Techniques.** `update_with_feedforward()` adds the model gravity torque `m*g*l*cos(q)` (with a
deliberate 5 % model error). Derivative on error is used because the reference is a smooth
profile: derivative on measurement would add damping against the motion and slow tracking.

**Lesson.** Gravity feedforward cuts the tracking error during the move by more than half; the
integrator is still needed for model error and the unknown payload.

## `gain_scheduling_payload.cpp`

**Scenario.** A SCARA joint moving in the horizontal plane carries 0 to 4 kg at the tip, so its
inertia changes by about 5x.

**Techniques.** Gains are computed for each payload to place the closed-loop poles at the same
frequency (`kp = J*wn^2`, `kd = 2*zeta*J*wn`) and stored in a `GainScheduledPidController`
indexed by the payload the gripper reports.

**Lesson.** Gains tuned for the empty arm overshoot by about 35 % with the heavy payload; the
scheduled controller stays below 1 %.

## `mobile_robot_navigation.cpp`

**Scenario.** A differential-drive robot (with first-order wheel-speed lag) first turns from
+170 to -170 degrees, then drives a 2 m square of waypoints.

**Techniques.** `continuous_input = true` makes the heading controller take the short way across
the +/-180 degree seam. For waypoints, a heading PID steers toward the bearing, a distance P
loop sets speed, speed is scaled down with heading error so the robot turns before it drives,
and a `SlewRateLimiter` limits acceleration and braking.

## `drone_altitude_hold.cpp`

**Scenario.** A 1.2 kg quadrotor takes off to 10 m, hovers, and drops a 0.3 kg package at
t = 15 s.

**Techniques.** The hover thrust `m*g` is the static feedforward (`feedforward.static_gain`), so
the PID only handles deviations. Thrust is limited to `[0, 25] N` (propellers cannot pull down),
its rate is limited to 80 N/s to protect the ESCs, and the altitude setpoint is slew-rate
limited for a smooth climb. After the drop the integrator re-trims the hover thrust.

## `relay_autotune.cpp`

**Scenario.** Tune a motor speed loop without a model.

**Techniques.** The motor is brought to an operating point, then the relay toggles the voltage
by +/-2 V around it. Once consecutive oscillation periods agree, the tuner reports the ultimate
gain `Ku` and period `Tu`. The example prints gains for several rules and validates a
Tyreus-Luyben PI controller with a closed-loop step.

**Notes.** Relay tuning needs a plant whose phase reaches -180 degrees: dead time or at least
three lags. Here the encoder speed filter provides that lag, as it does on most real drives.
Use hysteresis larger than the measurement noise.

## `sensor_noise_filtering.cpp`

**Scenario.** A joint is read through a 12-bit encoder with one count of noise.

**Lesson.** Differentiating a quantized signal produces large spikes; with `kd = 4` the
unfiltered derivative makes the torque command chatter by about 10 N*m RMS. A 10 ms derivative
filter reduces that by more than 15x while holding position equally well. The noise generator is
a fixed-seed LCG, so results are identical on every platform.

## `step_response_analysis.cpp`

Compares P, PI, and PID on a two-lag process with the offline metrics helpers: P leaves the
expected steady-state error `1/(1 + Kp)`, the integrator removes it, and derivative action
speeds up settling.

## `basic_pid.cpp`

The smallest complete program: configure, validate, construct, update, read the state.

## `cmake_consumer_example`

A standalone CMake project that consumes an installed PID++ package:

```cmake
find_package(PIDPP CONFIG REQUIRED)
target_link_libraries(pidpp_consumer PRIVATE pidpp::pidpp)
```

```sh
cmake --install build --prefix "$PWD/install"
cmake -S examples/cmake_consumer_example -B consumer -DCMAKE_PREFIX_PATH="$PWD/install"
cmake --build consumer && ./consumer/pidpp_consumer
```
