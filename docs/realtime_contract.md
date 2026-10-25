# Real-Time Contract

## Guaranteed

The following functions are `noexcept`, do not allocate, do not log, do not use virtual
dispatch, and perform bounded work:

| Function | Complexity |
| --- | --- |
| `PidController<T>::update()` / `update_with_feedforward()` | O(1) |
| `CascadedPidController<T>::update()` | O(1) |
| `GainScheduledPidController<T, N>::update()` | O(N) |
| `RelayAutotuner<T>::update()` | O(1) |
| `TrapezoidalProfile<T>::plan()` / `sample()` | O(1) |
| `LowPassFilter`, `MovingAverage`, `SlewRateLimiter` `update()` | O(1) |
| `PidController<T>::set_gains()` / `set_config()` / `reset()` | O(1) |

Invalid `dt` (zero, negative, NaN, infinite), setpoint, measurement, or feedforward input makes a
controller return its previous output and set `state().last_dt_valid = false`.

The unit tests replace global `operator new` and assert that 1000 consecutive `update()` calls
perform zero allocations.

## Outside the contract

These modules are useful SDK components but are intended for setup, analysis, or simulation:

- `validate()` functions (cheap, but call them before entering the loop);
- `GainScheduledPidController::set_schedule()` (copies and validates the table);
- `metrics`: offline response analysis over spans;
- `simulation`: plant models for examples and tests;
- examples, benchmarks, and the Control Lab.

## Integration guidance

- Validate configuration before entering a control loop and refuse to start on error.
- Keep logging, allocation, I/O, and parameter loading outside the loop; copy `state()` into a
  lock-free buffer if you need telemetry.
- Measure `dt` from a monotonic clock and pass the real value.
- Apply runtime gain changes at a controlled synchronization point (for example between two
  cycles, from a double buffer).
- `PIDPP_EMBEDDED_PROFILE=ON` disables tests, examples, benchmarks, and docs for firmware builds.
