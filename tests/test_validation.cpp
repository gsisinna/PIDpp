#include <catch2/catch_test_macros.hpp>

#include <limits>
#include <string_view>
#include <type_traits>

#include <pidpp/pidpp.hpp>

TEST_CASE("result and error are small value types")
{
    constexpr auto ok = pidpp::Result::ok();
    constexpr auto error = pidpp::Result::error(pidpp::ErrorCode::InvalidConfiguration, "bad config");

    STATIC_REQUIRE(std::is_trivially_copyable_v<pidpp::Error>);
    STATIC_REQUIRE(std::is_trivially_copyable_v<pidpp::Result>);
    REQUIRE(ok);
    REQUIRE_FALSE(error);
    REQUIRE(error.code() == pidpp::ErrorCode::InvalidConfiguration);
    REQUIRE(error.error().message[0] == 'b');
}

TEST_CASE("version API reports SDK version")
{
    REQUIRE(PIDPP_VERSION_MAJOR == 0);
    REQUIRE(PIDPP_VERSION_MINOR == 1);
    REQUIRE(PIDPP_VERSION_PATCH == 0);
    REQUIRE(pidpp::version_major() == 0);
    REQUIRE(pidpp::version_minor() == 1);
    REQUIRE(pidpp::version_patch() == 0);
    REQUIRE(std::string_view{pidpp::version_string()} == "0.1.0");
    REQUIRE(std::string_view{pidpp::version()} == "0.1.0");
}

TEST_CASE("default PID config is valid")
{
    pidpp::PidConfig<double> config{};

    REQUIRE(config.validate());
}

TEST_CASE("negative gain is invalid")
{
    pidpp::PidConfig<double> config{};
    config.kp = -1.0;

    const auto result = config.validate();

    REQUIRE_FALSE(result);
    REQUIRE(result.code() == pidpp::ErrorCode::InvalidGain);
}

TEST_CASE("inverted output range is invalid")
{
    pidpp::PidConfig<double> config{};
    config.output_min = 1.0;
    config.output_max = -1.0;

    const auto result = config.validate();

    REQUIRE_FALSE(result);
    REQUIRE(result.code() == pidpp::ErrorCode::InvalidOutputLimits);
}

TEST_CASE("non finite gain is invalid")
{
    pidpp::PidConfig<double> config{};
    config.kd = std::numeric_limits<double>::infinity();

    const auto result = config.validate();

    REQUIRE_FALSE(result);
    REQUIRE(result.code() == pidpp::ErrorCode::InvalidGain);
}

TEST_CASE("time step validation rejects zero")
{
    const auto result = pidpp::validate_time_step(0.0);

    REQUIRE_FALSE(result);
    REQUIRE(result.code() == pidpp::ErrorCode::InvalidDt);
}
