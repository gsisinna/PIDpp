#include <catch2/catch_test_macros.hpp>

#include <limits>
#include <string_view>
#include <type_traits>

#include <pidpp/pidpp.hpp>

TEST_CASE("result and error are small value types") {
    constexpr auto ok = pidpp::Result::ok();
    constexpr auto error =
        pidpp::Result::error(pidpp::ErrorCode::InvalidConfiguration, "bad config");

    STATIC_REQUIRE(std::is_trivially_copyable_v<pidpp::Error>);
    STATIC_REQUIRE(std::is_trivially_copyable_v<pidpp::Result>);
    REQUIRE(ok);
    REQUIRE_FALSE(error);
    REQUIRE(error.code() == pidpp::ErrorCode::InvalidConfiguration);
    REQUIRE(error.error().message[0] == 'b');
}

TEST_CASE("version API reports SDK version") {
    REQUIRE(pidpp::version_major() == PIDPP_VERSION_MAJOR);
    REQUIRE(pidpp::version_minor() == PIDPP_VERSION_MINOR);
    REQUIRE(pidpp::version_patch() == PIDPP_VERSION_PATCH);
    // PIDPP_EXPECTED_VERSION is injected by CMake from project(VERSION ...).
    REQUIRE(std::string_view{pidpp::version_string()} == PIDPP_EXPECTED_VERSION);
    REQUIRE(std::string_view{pidpp::version()} == PIDPP_EXPECTED_VERSION);
}

TEST_CASE("default PID config is valid") {
    pidpp::PidConfig<double> config{};

    REQUIRE(config.validate());
}

TEST_CASE("negative gain is invalid") {
    pidpp::PidConfig<double> config{};
    config.kp = -1.0;

    const auto result = config.validate();

    REQUIRE_FALSE(result);
    REQUIRE(result.code() == pidpp::ErrorCode::InvalidGain);
}

TEST_CASE("inverted output range is invalid") {
    pidpp::PidConfig<double> config{};
    config.output_min = 1.0;
    config.output_max = -1.0;

    const auto result = config.validate();

    REQUIRE_FALSE(result);
    REQUIRE(result.code() == pidpp::ErrorCode::InvalidOutputLimits);
}

TEST_CASE("non finite gain is invalid") {
    pidpp::PidConfig<double> config{};
    config.kd = std::numeric_limits<double>::infinity();

    const auto result = config.validate();

    REQUIRE_FALSE(result);
    REQUIRE(result.code() == pidpp::ErrorCode::InvalidGain);
}

TEST_CASE("time step validation rejects zero") {
    const auto result = pidpp::validate_time_step(0.0);

    REQUIRE_FALSE(result);
    REQUIRE(result.code() == pidpp::ErrorCode::InvalidDt);
}

TEST_CASE("time step validation rejects non finite and negative values") {
    REQUIRE_FALSE(pidpp::validate_time_step(std::numeric_limits<double>::quiet_NaN()));
    REQUIRE_FALSE(pidpp::validate_time_step(-0.001));
    REQUIRE(pidpp::validate_time_step(0.001));
}

TEST_CASE("extended config fields are validated") {
    pidpp::PidConfig<double> config{};

    config.error_deadband = -1.0;
    REQUIRE(config.validate().code() == pidpp::ErrorCode::InvalidConfiguration);
    config.error_deadband = 0.0;

    config.output_rate_limit = -1.0;
    REQUIRE(config.validate().code() == pidpp::ErrorCode::InvalidConfiguration);
    config.output_rate_limit = 0.0;

    config.continuous_input = true;
    config.input_range_min = 1.0;
    config.input_range_max = 1.0;
    REQUIRE(config.validate().code() == pidpp::ErrorCode::InvalidConfiguration);
    config.input_range_max = 2.0;
    REQUIRE(config.validate());

    config.feedforward.friction_gain = std::numeric_limits<double>::infinity();
    REQUIRE(config.validate().code() == pidpp::ErrorCode::InvalidConfiguration);
}

TEST_CASE("pid gains round trip through the config") {
    pidpp::PidConfig<double> config{};
    config.set_gains({1.0, 2.0, 3.0});

    const auto gains = config.gains();

    REQUIRE(gains.kp == 1.0);
    REQUIRE(gains.ki == 2.0);
    REQUIRE(gains.kd == 3.0);
    REQUIRE(config.validate());
}

TEST_CASE("float controllers are supported") {
    pidpp::PidConfig<float> config{.kp = 2.0F, .output_min = -5.0F, .output_max = 5.0F};
    REQUIRE(config.validate());
    pidpp::PidController<float> controller{config};

    REQUIRE(controller.update(1.0F, 0.0F, 0.01F) == 2.0F);
}
