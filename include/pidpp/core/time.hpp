#pragma once

#include <concepts>

#include <pidpp/core/numeric.hpp>
#include <pidpp/core/result.hpp>

namespace pidpp {

template <std::floating_point T>
[[nodiscard]] Result validate_time_step(T dt) noexcept
{
    if (!is_finite(dt)) {
        return Result::error(ErrorCode::InvalidDt, "dt must be finite");
    }
    if (dt <= T{0}) {
        return Result::error(ErrorCode::InvalidDt, "dt must be positive");
    }
    return Result::ok();
}

} // namespace pidpp
