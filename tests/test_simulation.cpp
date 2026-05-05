#include <catch2/catch_test_macros.hpp>
#include <catch2/matchers/catch_matchers_floating_point.hpp>

#include <pidpp/pidpp.hpp>

using Catch::Matchers::WithinAbs;

TEST_CASE("first-order plant moves toward steady state")
{
    pidpp::FirstOrderPlant<double> plant{2.0, 1.0};

    const double y0 = plant.update(1.0, 0.1);
    const double y1 = plant.update(1.0, 0.1);

    REQUIRE(y0 > 0.0);
    REQUIRE(y1 > y0);
    REQUIRE(y1 < 2.0);
}

TEST_CASE("DC motor speed increases under positive voltage")
{
    pidpp::DcMotor<double> motor{2.0, 0.01, 0.1, 0.1, 0.01, 0.001};

    const double w0 = motor.update(6.0, 0.0, 0.001);
    double w1 = w0;
    for (int i = 0; i < 100; ++i) {
        w1 = motor.update(6.0, 0.0, 0.001);
    }

    REQUIRE(w0 >= 0.0);
    REQUIRE(w1 > w0);
    REQUIRE(motor.current() > 0.0);
}

TEST_CASE("mass spring damper responds to force")
{
    pidpp::MassSpringDamper<double> plant{1.0, 0.2, 2.0};

    const double p0 = plant.update(1.0, 0.01);
    const double p1 = plant.update(1.0, 0.01);

    REQUIRE(p0 > 0.0);
    REQUIRE(p1 > p0);
}
