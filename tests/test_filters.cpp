#include <catch2/catch_test_macros.hpp>
#include <catch2/matchers/catch_matchers_floating_point.hpp>

#include <pidpp/pidpp.hpp>

using Catch::Matchers::WithinAbs;

TEST_CASE("low pass filter initializes from first sample")
{
    pidpp::LowPassFilter<double> filter{0.1};

    const double value = filter.update(5.0, 0.01);

    REQUIRE_THAT(value, WithinAbs(5.0, 1.0e-12));
}

TEST_CASE("low pass filter smooths subsequent samples")
{
    pidpp::LowPassFilter<double> filter{0.1};

    static_cast<void>(filter.update(0.0, 0.01));
    const double value = filter.update(10.0, 0.1);

    REQUIRE_THAT(value, WithinAbs(5.0, 1.0e-12));
}

TEST_CASE("moving average uses fixed window")
{
    pidpp::MovingAverage<double, 3> average;

    REQUIRE_THAT(average.update(1.0), WithinAbs(1.0, 1.0e-12));
    REQUIRE_THAT(average.update(2.0), WithinAbs(1.5, 1.0e-12));
    REQUIRE_THAT(average.update(3.0), WithinAbs(2.0, 1.0e-12));
    REQUIRE_THAT(average.update(4.0), WithinAbs(3.0, 1.0e-12));
}
