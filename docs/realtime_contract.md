# Real-Time Contract

This contract applies to `pidpp::PidController<T>::update()`.

## Guaranteed

- `update()` is `noexcept`.
- `update()` does not allocate dynamic memory.
- `update()` performs deterministic O(1) work.
- `update()` does not log.
- `update()` does not use virtual dispatch.
- Invalid `dt`, setpoint, or measurement returns the previous output and records the condition in controller state.

## Outside The Contract

The following modules are useful SDK components but are not guaranteed hard real-time:

- `metrics`: offline response analysis.
- `autotune`: tuning workflows.
- `simulation`: plant examples and deterministic tests.
- examples and benchmarks.

## Integration Guidance

Validate configuration before entering a control loop. Keep logging, allocation, I/O, and parameter loading outside the loop. Apply runtime gain updates at controlled synchronization points in your robot software.

`PIDPP_EMBEDDED_PROFILE=ON` selects embedded-friendly build defaults by disabling nonessential build targets.
