#pragma once

#include <concepts>

#include <pidpp/controllers/feedforward.hpp>

namespace pidpp::experimental {

template <std::floating_point T> struct FeedforwardControllerConfig {
    FeedforwardConfig<T> feedforward{};
};

} // namespace pidpp::experimental
