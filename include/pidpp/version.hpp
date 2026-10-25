#pragma once

#define PIDPP_VERSION_MAJOR 0
#define PIDPP_VERSION_MINOR 2
#define PIDPP_VERSION_PATCH 0
#define PIDPP_VERSION_STRING "0.2.0"

namespace pidpp {

[[nodiscard]] constexpr int version_major() noexcept {
    return PIDPP_VERSION_MAJOR;
}

[[nodiscard]] constexpr int version_minor() noexcept {
    return PIDPP_VERSION_MINOR;
}

[[nodiscard]] constexpr int version_patch() noexcept {
    return PIDPP_VERSION_PATCH;
}

[[nodiscard]] constexpr const char* version_string() noexcept {
    return PIDPP_VERSION_STRING;
}

} // namespace pidpp
