# CMake Integration

PID++ exports a modern CMake package.

## Install

```sh
cmake -S . -B build -DCMAKE_BUILD_TYPE=Release -DPIDPP_BUILD_TESTS=OFF
cmake --build build
cmake --install build --prefix /opt/pidpp
```

## Consume

```cmake
find_package(PIDPP CONFIG REQUIRED)
target_link_libraries(my_robot PRIVATE pidpp::pidpp)
```

Configure your downstream project with:

```sh
cmake -S . -B build -DCMAKE_PREFIX_PATH=/opt/pidpp
```

The exported target carries include directories and C++20 requirements. PID++ warning and sanitizer options are private to PID++ targets and are not forced onto downstream consumers.

## Package Metadata

The install step generates:

- `PIDPPConfig.cmake`
- `PIDPPConfigVersion.cmake`
- `PIDPPTargets.cmake`
- `pidpp.pc`
