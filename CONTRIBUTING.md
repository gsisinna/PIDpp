# Contributing

Thanks for helping improve PID++ Control Engine.

## Development Loop

```sh
cmake -S . -B build -DPIDPP_BUILD_TESTS=ON -DPIDPP_BUILD_EXAMPLES=ON
cmake --build build
ctest --test-dir build --output-on-failure
```

Keep controller hot paths allocation-free, `noexcept`, and deterministic. Prefer small, focused pull requests with tests that cover behavior rather than implementation details.

## Style

- C++20.
- Public namespace: `pidpp`.
- Types use `PascalCase`; functions and variables use `snake_case`.
- Keep public API headers documented.
- Experimental APIs live under `include/pidpp/experimental` and `pidpp::experimental`.
- Internal helpers live under `include/pidpp/internal` and are not stable API.

## Compatibility

Before 1.0, source compatibility is a goal but not a guarantee. ABI stability is not promised before 1.0.
