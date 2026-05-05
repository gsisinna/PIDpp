include(FetchContent)

FetchContent_Declare(
    Catch2
    GIT_REPOSITORY https://github.com/catchorg/Catch2.git
    GIT_TAG v3.6.0
)
FetchContent_MakeAvailable(Catch2)

add_executable(pidpp_tests
    test_filters.cpp
    test_pid_controller.cpp
    test_simulation.cpp
    test_validation.cpp
)

target_link_libraries(pidpp_tests PRIVATE pidpp::pidpp Catch2::Catch2WithMain)

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
