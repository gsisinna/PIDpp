# API Design

PID++ separates public, experimental, and internal API surfaces.

## Stable Public API

The stable public API lives under:

- `include/pidpp/pidpp.hpp`
- `include/pidpp/version.hpp`
- `include/pidpp/core`
- `include/pidpp/controllers`
- `include/pidpp/filters`
- `include/pidpp/metrics`
- `include/pidpp/simulation`
- `include/pidpp/autotune`

Public APIs use explicit validation and value types. Hot controller paths avoid exceptions, allocation, logging, and virtual calls.

## Experimental API

Experimental extension points live under `include/pidpp/experimental` and namespace `pidpp::experimental`.

These APIs are intentionally incomplete and may change in any release. They document future directions for cascaded PID, gain scheduling, feedforward controllers, relay autotuning aliases, and lightweight MPC problem sizing.

## Internal API

Headers under `include/pidpp/internal` are not stable API. They may be installed because PID++ is header-first, but downstream code should not include them.

## Compatibility

PID++ uses semantic versioning expectations. During `0.x`, source compatibility is a goal but not guaranteed. `1.0` indicates a stable public API. ABI stability is not guaranteed before 1.0.
