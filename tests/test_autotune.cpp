#include <catch2/catch_test_macros.hpp>
#include <catch2/matchers/catch_matchers_floating_point.hpp>

#include <cmath>

#include <pidpp/pidpp.hpp>

using Catch::Matchers::WithinAbs;
using Catch::Matchers::WithinRel;

TEST_CASE("relay toggles around the setpoint with hysteresis") {
    pidpp::RelayAutotuner<double> relay{{.relay_amplitude = 2.0, .hysteresis = 0.1}};

    REQUIRE_THAT(relay.update(1.0, 0.0), WithinAbs(2.0, 1.0e-12));
    REQUIRE_THAT(relay.update(1.0, 1.05), WithinAbs(2.0, 1.0e-12));
    REQUIRE_THAT(relay.update(1.0, 1.2), WithinAbs(-2.0, 1.0e-12));
    REQUIRE_THAT(relay.update(1.0, 0.95), WithinAbs(-2.0, 1.0e-12));
    REQUIRE_THAT(relay.update(1.0, 0.8), WithinAbs(2.0, 1.0e-12));
}

TEST_CASE("relay autotune config validation") {
    REQUIRE(pidpp::RelayAutotuneConfig<double>{}.validate());
    REQUIRE_FALSE(pidpp::RelayAutotuneConfig<double>{.relay_amplitude = 0.0}.validate());
    REQUIRE_FALSE(pidpp::RelayAutotuneConfig<double>{.hysteresis = -1.0}.validate());
    REQUIRE_FALSE(pidpp::RelayAutotuneConfig<double>{.cycles = 0}.validate());
}

TEST_CASE("relay experiment identifies the ultimate point of a third-order plant") {
    // Three identical lags 1/(s+1)^3: Ku = 8 and Tu = 2*pi/sqrt(3) analytically.
    pidpp::FirstOrderPlant<double> p1{1.0, 1.0};
    pidpp::FirstOrderPlant<double> p2{1.0, 1.0};
    pidpp::FirstOrderPlant<double> p3{1.0, 1.0};
    pidpp::RelayAutotuner<double> tuner{{.relay_amplitude = 1.0, .cycles = 4, .timeout = 200.0}};

    constexpr double dt = 0.001;
    double output = 0.0;
    for (int i = 0; i < 200000 && !tuner.complete(); ++i) {
        const double y = p3.update(p2.update(p1.update(output, dt), dt), dt);
        output = tuner.update(0.0, y, dt);
    }

    REQUIRE(tuner.complete());
    const auto& result = tuner.result();
    REQUIRE(result.measured_cycles == 4);
    REQUIRE_THAT(result.ultimate_period, WithinRel(2.0 * 3.14159265358979 / std::sqrt(3.0), 0.05));
    // The describing function approximation is accurate to ~10-20 % here.
    REQUIRE_THAT(result.ultimate_gain, WithinRel(8.0, 0.2));

    const auto gains = tuner.gains(pidpp::TuningRule::ZieglerNicholsPid);
    REQUIRE_THAT(gains.kp, WithinRel(0.6 * result.ultimate_gain, 1.0e-12));
    REQUIRE(gains.validate());

    // After completion the relay rests at the bias.
    REQUIRE_THAT(tuner.update(0.0, 1.0, dt), WithinAbs(0.0, 1.0e-12));
}

TEST_CASE("relay experiment times out on a plant that never oscillates") {
    pidpp::RelayAutotuner<double> tuner{
        {.relay_amplitude = 1.0, .output_bias = 0.5, .timeout = 1.0}};

    for (int i = 0; i < 2000; ++i) {
        static_cast<void>(tuner.update(10.0, 0.0, 0.001));
    }

    REQUIRE_FALSE(tuner.complete());
    REQUIRE(tuner.result().timed_out);
    REQUIRE_THAT(tuner.state().output, WithinAbs(0.5, 1.0e-12));
    REQUIRE(tuner.gains(pidpp::TuningRule::ZieglerNicholsPid).kp == 0.0);
}

TEST_CASE("tuning rules follow the published tables") {
    const auto zn = pidpp::compute_tuning(10.0, 2.0, pidpp::TuningRule::ZieglerNicholsPid);
    REQUIRE_THAT(zn.kp, WithinAbs(6.0, 1.0e-12));
    REQUIRE_THAT(zn.ki, WithinAbs(6.0, 1.0e-12));
    REQUIRE_THAT(zn.kd, WithinAbs(1.5, 1.0e-12));

    const auto pi = pidpp::compute_tuning(10.0, 2.0, pidpp::TuningRule::ZieglerNicholsPi);
    REQUIRE_THAT(pi.kp, WithinAbs(4.5, 1.0e-12));
    REQUIRE_THAT(pi.ki, WithinAbs(2.7, 1.0e-12));
    REQUIRE_THAT(pi.kd, WithinAbs(0.0, 1.0e-12));

    const auto p = pidpp::compute_tuning(10.0, 2.0, pidpp::TuningRule::ZieglerNicholsP);
    REQUIRE_THAT(p.kp, WithinAbs(5.0, 1.0e-12));
    REQUIRE_THAT(p.ki, WithinAbs(0.0, 1.0e-12));

    const auto tl = pidpp::compute_tuning(10.0, 2.0, pidpp::TuningRule::TyreusLuybenPid);
    REQUIRE(tl.kp < zn.kp);
    REQUIRE(tl.ki < zn.ki);

    const auto none = pidpp::compute_tuning(-1.0, 2.0, pidpp::TuningRule::ZieglerNicholsPid);
    REQUIRE(none.kp == 0.0);
}
