cmake_minimum_required(VERSION 3.20)

project(PIDPP
    VERSION 0.1.0
    DESCRIPTION "PIDPP Control Engine"
    LANGUAGES CXX
)

option(PIDPP_EMBEDDED_PROFILE "Use embedded-friendly default build settings" OFF)

set(PIDPP_DEFAULT_BUILD_TESTS ON)
set(PIDPP_DEFAULT_BUILD_EXAMPLES ON)
set(PIDPP_DEFAULT_BUILD_BENCHMARKS OFF)
set(PIDPP_DEFAULT_BUILD_DOCS OFF)
set(PIDPP_DEFAULT_BUILD_GUI OFF)

if(PIDPP_EMBEDDED_PROFILE)
    set(PIDPP_DEFAULT_BUILD_TESTS OFF)
    set(PIDPP_DEFAULT_BUILD_EXAMPLES OFF)
    set(PIDPP_DEFAULT_BUILD_BENCHMARKS OFF)
    set(PIDPP_DEFAULT_BUILD_DOCS OFF)
    set(PIDPP_DEFAULT_BUILD_GUI OFF)
endif()

option(PIDPP_BUILD_TESTS "Build PIDPP tests" ${PIDPP_DEFAULT_BUILD_TESTS})
option(PIDPP_BUILD_EXAMPLES "Build PIDPP examples" ${PIDPP_DEFAULT_BUILD_EXAMPLES})
option(PIDPP_BUILD_BENCHMARKS "Build PIDPP benchmarks" ${PIDPP_DEFAULT_BUILD_BENCHMARKS})
option(PIDPP_BUILD_GUI "Build PIDPP interactive GUI tool" ${PIDPP_DEFAULT_BUILD_GUI})
option(PIDPP_ENABLE_WARNINGS "Enable strict compiler warnings" ON)
option(PIDPP_ENABLE_SANITIZERS "Enable address and undefined behavior sanitizers" OFF)
option(PIDPP_BUILD_DOCS "Enable documentation-related targets when available" ${PIDPP_DEFAULT_BUILD_DOCS})
option(PIDPP_INSTALL "Install PIDPP targets, headers, and package metadata" ON)

include(GNUInstallDirs)
include(CMakePackageConfigHelpers)

add_library(pidpp
    src/version.cpp
)

add_library(pidpp::pidpp ALIAS pidpp)

target_compile_features(pidpp PUBLIC cxx_std_20)

target_include_directories(pidpp
    PUBLIC
        $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
        $<INSTALL_INTERFACE:${CMAKE_INSTALL_INCLUDEDIR}>
)

if(PIDPP_ENABLE_WARNINGS)
    if(MSVC)
        target_compile_options(pidpp PRIVATE /W4 /permissive-)
    else()
        target_compile_options(pidpp PRIVATE -Wall -Wextra -Wpedantic -Wconversion -Wsign-conversion)
    endif()
endif()

if(PIDPP_ENABLE_SANITIZERS)
    if(CMAKE_CXX_COMPILER_ID MATCHES "Clang|GNU")
        set(PIDPP_SANITIZER_COMPILE_OPTIONS -fsanitize=address,undefined -fno-omit-frame-pointer)
        set(PIDPP_SANITIZER_LINK_OPTIONS -fsanitize=address,undefined)
        target_compile_options(pidpp PRIVATE ${PIDPP_SANITIZER_COMPILE_OPTIONS})
        target_link_options(pidpp PRIVATE ${PIDPP_SANITIZER_LINK_OPTIONS})
    endif()
endif()

if(PIDPP_INSTALL)
    install(TARGETS pidpp
        EXPORT PIDPPTargets
        ARCHIVE DESTINATION ${CMAKE_INSTALL_LIBDIR}
        LIBRARY DESTINATION ${CMAKE_INSTALL_LIBDIR}
        RUNTIME DESTINATION ${CMAKE_INSTALL_BINDIR}
    )

    install(DIRECTORY include/ DESTINATION ${CMAKE_INSTALL_INCLUDEDIR})

    install(EXPORT PIDPPTargets
        FILE PIDPPTargets.cmake
        NAMESPACE pidpp::
        DESTINATION ${CMAKE_INSTALL_LIBDIR}/cmake/PIDPP
    )

    write_basic_package_version_file(
        "${CMAKE_CURRENT_BINARY_DIR}/PIDPPConfigVersion.cmake"
        VERSION ${PROJECT_VERSION}
        COMPATIBILITY SameMajorVersion
    )

    configure_package_config_file(
        "${CMAKE_CURRENT_SOURCE_DIR}/cmake/PIDPPConfig.cmake.in"
        "${CMAKE_CURRENT_BINARY_DIR}/PIDPPConfig.cmake"
        INSTALL_DESTINATION ${CMAKE_INSTALL_LIBDIR}/cmake/PIDPP
    )

    configure_file(
        "${CMAKE_CURRENT_SOURCE_DIR}/cmake/pidpp.pc.in"
        "${CMAKE_CURRENT_BINARY_DIR}/pidpp.pc"
        @ONLY
    )

    install(FILES
        "${CMAKE_CURRENT_BINARY_DIR}/PIDPPConfig.cmake"
        "${CMAKE_CURRENT_BINARY_DIR}/PIDPPConfigVersion.cmake"
        DESTINATION ${CMAKE_INSTALL_LIBDIR}/cmake/PIDPP
    )

    install(FILES
        "${CMAKE_CURRENT_BINARY_DIR}/pidpp.pc"
        DESTINATION ${CMAKE_INSTALL_LIBDIR}/pkgconfig
    )
endif()

if(PIDPP_BUILD_TESTS)
    enable_testing()
    add_subdirectory(tests)
endif()

if(PIDPP_BUILD_EXAMPLES)
    add_subdirectory(examples)
endif()

if(PIDPP_BUILD_BENCHMARKS)
    add_subdirectory(benchmarks)
endif()

if(PIDPP_BUILD_GUI)
    add_subdirectory(tools/gui)
endif()
