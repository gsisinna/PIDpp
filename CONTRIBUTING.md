# Contributing

Thanks for helping improve PID++ Control Engine.

## Development loop

```sh
cmake -S . -B build -DCMAKE_BUILD_TYPE=Debug -DPIDPP_ENABLE_SANITIZERS=ON
cmake --build build --parallel
ctest --test-dir build --output-on-failure
```

`ctest` runs the Catch2 unit tests and every example as a scenario test. Optional targets:

```sh
cmake -S . -B build -DPIDPP_BUILD_BENCHMARKS=ON -DPIDPP_BUILD_GUI=ON -DPIDPP_BUILD_DOCS=ON
```

## Before opening a pull request

- Format: `clang-format -i` with clang-format 17 (`pip install clang-format==17.0.6`). CI checks
  every file under `include src tests examples benchmarks tools`.
- Static analysis: `clang-tidy -p build src/version.cpp` must be clean (warnings are errors).
- Tests: add or update tests for every behavior change. New examples must check their own
  acceptance criteria and return non-zero on failure; register them in
  `examples/CMakeLists.txt`.
- Changelog: add an entry under a new or unreleased version in `CHANGELOG.md`.
- Version: when releasing, bump `project(VERSION ...)` in `CMakeLists.txt` and
  `include/pidpp/version.hpp` together (configure fails if they differ), plus `conanfile.py`
  and `vcpkg.json`.

## Design rules

- Controller hot paths stay allocation-free, `noexcept`, deterministic, and free of logging
  and virtual dispatch. The allocation test in `tests/test_pid_controller.cpp` enforces this
  for `PidController`.
- Configuration is validated explicitly through `validate()` returning `Result`; nothing
  throws.
- Prefer composition over inheritance and fixed-capacity storage over dynamic containers.
- Use SI units and seconds; document units in comments.

## Style

- C++20, LLVM-based style with 4-space indentation and 100 columns (`.clang-format`).
- Public namespace: `pidpp`. Types use `PascalCase`; functions and variables use `snake_case`.
- Document public API headers with Doxygen comments.
- Experimental APIs live under `include/pidpp/experimental` and `pidpp::experimental`.
- Internal helpers live under `include/pidpp/internal` and are not stable API.

## Commit messages

Use [Conventional Commits](https://www.conventionalcommits.org/) prefixes (`feat:`, `fix:`,
`docs:`, `ci:`, `build:`, `chore:`) with a body that explains why the change is needed.

## Compatibility

Before 1.0, source compatibility is a goal but not a guarantee. ABI stability is not promised
before 1.0.
