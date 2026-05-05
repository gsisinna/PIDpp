include(FetchContent)

FetchContent_Declare(
    benchmark
    GIT_REPOSITORY https://github.com/google/benchmark.git
    GIT_TAG v1.8.3
)
set(BENCHMARK_ENABLE_TESTING OFF CACHE BOOL "" FORCE)
FetchContent_MakeAvailable(benchmark)

add_executable(pidpp_update_benchmark update_latency.cpp)
target_link_libraries(pidpp_update_benchmark PRIVATE pidpp::pidpp benchmark::benchmark_main)

if(PIDPP_ENABLE_SANITIZERS AND CMAKE_CXX_COMPILER_ID MATCHES "Clang|GNU")
    target_compile_options(pidpp_update_benchmark PRIVATE ${PIDPP_SANITIZER_COMPILE_OPTIONS})
    target_link_options(pidpp_update_benchmark PRIVATE ${PIDPP_SANITIZER_LINK_OPTIONS})
endif()
