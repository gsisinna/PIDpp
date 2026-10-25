# CMake Integration

PID++ exports a modern CMake package (`PIDPP`, target `pidpp::pidpp`).

## Install

```sh
cmake -S . -B build -DCMAKE_BUILD_TYPE=Release -DPIDPP_BUILD_TESTS=OFF -DPIDPP_BUILD_EXAMPLES=OFF
cmake --build build
cmake --install build --prefix /opt/pidpp
```

## Consume an installed package

```cmake
find_package(PIDPP 0.2 CONFIG REQUIRED)
target_link_libraries(my_robot PRIVATE pidpp::pidpp)
```

```sh
cmake -S . -B build -DCMAKE_PREFIX_PATH=/opt/pidpp
```

The version file uses `SameMajorVersion` compatibility. See
[`examples/cmake_consumer_example`](../examples/cmake_consumer_example) for a complete project;
CI builds it against a fresh install on every push.

## Consume with FetchContent

```cmake
include(FetchContent)
FetchContent_Declare(
    pidpp
    GIT_REPOSITORY https://github.com/gsisinna/PIDpp.git
    GIT_TAG main  # pin a commit or tag in production
)
set(PIDPP_BUILD_TESTS OFF CACHE BOOL "" FORCE)
set(PIDPP_BUILD_EXAMPLES OFF CACHE BOOL "" FORCE)
FetchContent_MakeAvailable(pidpp)

target_link_libraries(my_robot PRIVATE pidpp::pidpp)
```

`add_subdirectory(path/to/PIDpp)` works the same way.

## pkg-config

```sh
PKG_CONFIG_PATH=/opt/pidpp/lib/pkgconfig pkg-config --cflags --libs pidpp
```

Remember to compile with `-std=c++20`.

## What the target carries

The exported target carries the include directory and the C++20 requirement. PID++ warning and
sanitizer options are private to PID++ targets and are never forced onto consumers.

## Cross-compiling for embedded targets

The library has no dependencies beyond the C++ standard library (`<cmath>`, `<array>`,
`<span>`, `<numbers>`, `<algorithm>`). With a cross toolchain file:

```sh
cmake -S . -B build-arm -DCMAKE_TOOLCHAIN_FILE=arm-none-eabi.cmake -DPIDPP_EMBEDDED_PROFILE=ON
```

Use `float` instantiations on MCUs with a single-precision FPU.

## Optional GUI tool

`PIDPP_BUILD_GUI=ON` builds `pidpp_control_lab` (GLFW, Dear ImGui, and ImPlot are fetched with
`FetchContent`). It is off by default and is not installed with the SDK. On Linux, install the
X11/OpenGL development packages first:

```sh
sudo apt-get install libgl1-mesa-dev libx11-dev libxrandr-dev libxinerama-dev libxcursor-dev libxi-dev
```

## Package metadata

The install step generates `PIDPPConfig.cmake`, `PIDPPConfigVersion.cmake`,
`PIDPPTargets.cmake`, and `pidpp.pc`.
