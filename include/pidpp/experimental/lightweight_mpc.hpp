#pragma once

#include <cstddef>

namespace pidpp::experimental {

template <std::size_t States, std::size_t Inputs, std::size_t Horizon>
struct LightweightMpcProblemSize {
    static constexpr std::size_t states = States;
    static constexpr std::size_t inputs = Inputs;
    static constexpr std::size_t horizon = Horizon;
};

} // namespace pidpp::experimental
