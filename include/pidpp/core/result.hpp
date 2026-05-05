#pragma once

namespace pidpp {

/**
 * Stable SDK error codes returned by validation and non-throwing setup APIs.
 */
enum class ErrorCode {
    None,
    InvalidDt,
    InvalidGain,
    InvalidOutputLimits,
    InvalidFilterTimeConstant,
    InvalidConfiguration,
    NotInitialized,
    UnsupportedOperation
};

/**
 * Lightweight error object.
 *
 * The message is a pointer to static storage. Result and Error do not allocate
 * and are suitable for validation paths used before entering a real-time loop.
 */
struct Error {
    ErrorCode code{ErrorCode::None};
    const char* message{""};
};

class Result {
  public:
    constexpr Result() noexcept = default;

    constexpr Result(ErrorCode code, const char* message) noexcept : error_{code, message} {}

    [[nodiscard]] constexpr explicit operator bool() const noexcept {
        return error_.code == ErrorCode::None;
    }

    [[nodiscard]] constexpr ErrorCode code() const noexcept {
        return error_.code;
    }

    [[nodiscard]] constexpr const char* message() const noexcept {
        return error_.message;
    }

    [[nodiscard]] constexpr Error error() const noexcept {
        return error_;
    }

    [[nodiscard]] static constexpr Result ok() noexcept {
        return {};
    }

    [[nodiscard]] static constexpr Result error(ErrorCode code, const char* message) noexcept {
        return {code, message};
    }

  private:
    Error error_{};
};

} // namespace pidpp
