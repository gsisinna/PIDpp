// Differential-drive mobile robot: heading control and waypoint following.
//
// Part 1 turns the robot from +170 deg to -170 deg. With continuous input the
// heading PID takes the short +20 deg turn instead of spinning -340 deg.
//
// Part 2 drives a square of waypoints. A heading PID steers toward each
// waypoint while a distance PID sets forward speed; forward speed is reduced
// when the heading error is large so the robot turns before it drives, and a
// slew-rate limiter keeps wheel acceleration gentle.
//
// Run with `--csv path.csv` to export the trajectory.

#include <algorithm>
#include <array>
#include <cmath>
#include <numbers>

#include <pidpp/pidpp.hpp>

#include "common/example_support.hpp"

namespace {

constexpr double dt = 0.01; // 100 Hz navigation loop

pidpp::PidConfig<double> heading_config() {
    return {.kp = 4.0,
            .ki = 0.2,
            .kd = 0.15,
            .output_min = -2.5, // rad/s yaw-rate limit
            .output_max = 2.5,
            .integral_min = -0.5,
            .integral_max = 0.5,
            .anti_windup = pidpp::AntiWindupStrategy::Clamping,
            .derivative_mode = pidpp::DerivativeMode::OnMeasurement,
            .derivative_filter_tau = 0.05,
            .continuous_input = true}; // default range [-pi, pi)
}

} // namespace

int main(int argc, char** argv) {
    example::CsvLog csv{argc, argv, {"part", "t", "x", "y", "heading", "v_cmd", "w_cmd"}};
    bool ok = true;

    // --- Part 1: heading step across the +/-180 deg seam -------------------
    {
        pidpp::PidController<double> heading{heading_config()};
        pidpp::DifferentialDrive<double> robot{0.1, 0.08};
        robot.reset({.heading = pidpp::deg_to_rad(170.0)});
        const double target = pidpp::deg_to_rad(-170.0);

        double w_cmd = 0.0;
        double min_abs_heading = std::numbers::pi;
        for (int i = 0; i < 300; ++i) {
            const auto& pose = robot.update(0.0, w_cmd, dt);
            w_cmd = heading.update(target, pose.heading, dt);
            min_abs_heading = std::min(min_abs_heading, std::abs(pose.heading));
            csv.row({1.0, i * dt, pose.x, pose.y, pose.heading, 0.0, w_cmd});
        }

        const double final_error = pidpp::angle_difference(target, robot.pose().heading);
        example::title("Heading step from +170 deg to -170 deg");
        example::print_value("final heading", pidpp::rad_to_deg(robot.pose().heading), "deg");
        example::print_value("final heading error", pidpp::rad_to_deg(final_error), "deg");
        example::print_value("closest approach to 0 deg", pidpp::rad_to_deg(min_abs_heading),
                             "deg");
        ok &= example::check(std::abs(final_error) < pidpp::deg_to_rad(0.5),
                             "reaches the target heading");
        ok &= example::check(min_abs_heading > pidpp::deg_to_rad(160.0),
                             "turns the short way across the seam");
    }

    // --- Part 2: square waypoint path ---------------------------------------
    {
        pidpp::PidController<double> heading{heading_config()};
        pidpp::PidController<double> distance{{.kp = 1.2,
                                               .output_min = 0.0,    // no reversing
                                               .output_max = 0.8}};  // m/s
        pidpp::SlewRateLimiter<double> speed_limiter{0.5, 2.0, 0.0}; // accel / brake m/s^2
        pidpp::DifferentialDrive<double> robot{0.1, 0.08};

        constexpr std::array<std::array<double, 2>, 4> waypoints{
            {{2.0, 0.0}, {2.0, 2.0}, {0.0, 2.0}, {0.0, 0.0}}};
        constexpr double tolerance = 0.05; // m

        std::size_t reached = 0;
        double v_cmd = 0.0;
        double w_cmd = 0.0;
        double t = 0.0;
        for (int i = 0; i < 6000 && reached < waypoints.size(); ++i, t += dt) {
            const auto& pose = robot.update(v_cmd, w_cmd, dt);
            const auto& goal = waypoints[reached];
            const double dx = goal[0] - pose.x;
            const double dy = goal[1] - pose.y;
            const double range = std::hypot(dx, dy);
            if (range < tolerance) {
                ++reached;
                continue;
            }

            const double bearing = std::atan2(dy, dx);
            w_cmd = heading.update(bearing, pose.heading, dt);
            const double heading_error = std::abs(pidpp::angle_difference(bearing, pose.heading));
            // Scale speed down as the heading error grows; stop above 60 deg.
            const double alignment =
                std::max(0.0, std::cos(std::min(heading_error * 1.5, std::numbers::pi / 2.0)));
            v_cmd = speed_limiter.update(distance.update(range, 0.0, dt) * alignment, dt);
            csv.row({2.0, t, pose.x, pose.y, pose.heading, v_cmd, w_cmd});
        }

        example::title("Square waypoint path (2 m x 2 m)");
        example::print_value("waypoints reached", static_cast<double>(reached));
        example::print_value("time", t, "s");
        example::print_value("final x", robot.pose().x, "m");
        example::print_value("final y", robot.pose().y, "m");
        ok &= example::check(reached == waypoints.size(), "visits every waypoint");
        ok &= example::check(t < 40.0, "completes the square within 40 s");
    }

    return example::finish(ok);
}
