include(FetchContent)

FetchContent_Declare(
    Catch2
    GIT_REPOSITORY https://github.com/catchorg/Catch2.git
    GIT_TAG v3.6.0
    GIT_SHALLOW TRUE
)
# Build Catch2 with the same language standard as the tests. Otherwise
# compilers that default to C++14 (MSVC, AppleClang) compile the Catch2
# library without std::string_view support and linking the tests fails.
set(CMAKE_CXX_STANDARD 20)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
FetchContent_MakeAvailable(Catch2)

add_executable(pidpp_tests
    test_autotune.cpp
    test_controllers_advanced.cpp
    test_filters.cpp
    test_motion_metrics.cpp
    test_pid_controller.cpp
    test_simulation.cpp
    test_validation.cpp
)

target_link_libraries(pidpp_tests PRIVATE pidpp::pidpp Catch2::Catch2WithMain)
target_compile_definitions(pidpp_tests PRIVATE PIDPP_EXPECTED_VERSION="${PROJECT_VERSION}")

if(PIDPP_ENABLE_WARNINGS)
    if(MSVC)
        target_compile_options(pidpp_tests PRIVATE /W4 /permissive-)
    else()
        target_compile_options(pidpp_tests PRIVATE -Wall -Wextra -Wpedantic -Wconversion -Wsign-conversion)
    endif()
endif()

if(PIDPP_ENABLE_SANITIZERS AND CMAKE_CXX_COMPILER_ID MATCHES "Clang|GNU")
    target_compile_options(pidpp_tests PRIVATE ${PIDPP_SANITIZER_COMPILE_OPTIONS})
    target_link_options(pidpp_tests PRIVATE ${PIDPP_SANITIZER_LINK_OPTIONS})
endif()

include(Catch)
catch_discover_tests(pidpp_tests)
