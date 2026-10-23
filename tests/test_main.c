#include <math.h>
#include <stdio.h>
#include "config.h"
#include "moving_average.h"
#include "pi_controller.h"
#include "simulation.h"
#include "wheel_model.h"

static int failures = 0;
static int checks = 0;

/* 不用assert：即使Release构建定义NDEBUG，测试仍然执行。 */
static void check(int passed, const char *description)
{
    ++checks;
    printf("[%s] %s\n", passed ? "PASS" : "FAIL", description);
    if (!passed) {
        ++failures;
    }
}

static int near(double actual, double expected, double tolerance)
{
    return isfinite(actual) && fabs(actual - expected) <= tolerance;
}

static void test_moving_average(void)
{
    MovingAverage filter;
    double value;
    int i;
    moving_average_init(&filter);
    check(near(moving_average_update(&filter, 10.0), 10.0, 1e-12),
          "filter startup uses one available sample");
    check(near(moving_average_update(&filter, 20.0), 15.0, 1e-12),
          "filter startup uses two available samples");
    (void)moving_average_update(&filter, 30.0);
    (void)moving_average_update(&filter, 40.0);
    check(near(moving_average_update(&filter, 50.0), 30.0, 1e-12),
          "five samples produce mean 30");
    check(near(moving_average_update(&filter, 60.0), 40.0, 1e-12),
          "sixth sample evicts oldest sample");
    value = 0.0;
    for (i = 0; i < 100; ++i) {
        value = moving_average_update(&filter, -7.0);
    }
    check(near(value, -7.0, 1e-12), "filter handles negative values and repeated wraparound");
    moving_average_init(&filter);
    check(near(moving_average_update(&filter, 8.0), 8.0, 1e-12),
          "filter reset discards old history");
}

static void test_pi(void)
{
    PiController pi;
    double output;
    int i;
    pi_init(&pi, 0.0, 2.0, 0.05, -10.0, 10.0);
    check(near(pi_update(&pi, 3.0, 0.0), 0.3, 1e-12),
          "PI integral includes 0.05-second sample period");
    check(near(pi_update(&pi, 0.0, 0.0), 0.3, 1e-12),
          "zero error retains integral effort");

    pi_init(&pi, 0.01, 0.1, 0.05, -0.04, 0.04);
    output = 0.0;
    for (i = 0; i < 1000; ++i) {
        output = pi_update(&pi, 1000.0, 0.0);
    }
    check(near(output, 0.04, 1e-12) && pi.saturated,
          "positive command is limited to +0.04 Nm");
    check(near(pi.integral_nm, 0.0, 1e-12), "positive saturation does not accumulate integral");
    for (i = 0; i < 1000; ++i) {
        output = pi_update(&pi, -1000.0, 0.0);
    }
    check(near(output, -0.04, 1e-12) && pi.saturated,
          "negative command is limited to -0.04 Nm");
    check(near(pi.integral_nm, 0.0, 1e-12), "negative saturation does not accumulate integral");
    check(near(pi_update(&pi, 0.0, 0.0), 0.0, 1e-12) && !pi.saturated,
          "controller leaves saturation when error is removed");

    pi_init(&pi, 0.001, 0.01, 0.05, -0.04, 0.04);
    pi.integral_nm = 0.03;
    (void)pi_update(&pi, -10.0, 0.0);
    check(pi.integral_nm < 0.03, "opposite error can reduce existing integral effort");
}

static void test_wheel_model(void)
{
    WheelModel wheel;
    int i;
    wheel_model_init(&wheel, 0.01, 0.0, 0.0);
    for (i = 0; i < 1000; ++i) {
        wheel_model_step(&wheel, 0.01, 0.001);
    }
    /* 独立物理预期：T/J=1 rad/s^2，1秒后omega=1 rad/s。 */
    check(near(wheel.speed_rad_s, 1.0, 1e-10), "constant torque obeys angular momentum balance");
    check(near(rad_s_to_rpm(wheel.speed_rad_s), 9.5492965855, 1e-9),
          "rad/s-to-rpm conversion has correct scale");
    check(near(rpm_to_rad_s(60.0), 6.283185307179586, 1e-12),
          "60 rpm equals 2*pi rad/s");

    wheel_model_init(&wheel, 0.005, 0.00002, 0.002);
    wheel_model_step(&wheel, 0.001, 0.001);
    check(near(wheel.speed_rad_s, 0.0, 1e-12), "sub-threshold torque cannot overcome static friction");
    wheel_model_step(&wheel, -0.003, 0.001);
    check(wheel.speed_rad_s < 0.0, "sufficient negative torque starts negative rotation");

    wheel.speed_rad_s = 0.1;
    for (i = 0; i < 1000; ++i) {
        wheel_model_step(&wheel, 0.0, 0.001);
    }
    check(near(wheel.speed_rad_s, 0.0, 1e-12),
          "unpowered friction stops the wheel without reversing it");
    wheel.speed_rad_s = -0.1;
    for (i = 0; i < 1000; ++i) {
        wheel_model_step(&wheel, 0.0, 0.001);
    }
    check(near(wheel.speed_rad_s, 0.0, 1e-12), "friction also stops negative rotation");
}

static void test_scenario(int noise_enabled)
{
    Simulation simulation;
    SimulationSample sample = {0};
    int i;
    int finite_and_bounded = 1;
    int regular_timing = 1;
    int hit_saturation = 0;
    double speed_before_disturbance = 0.0;
    double torque_before_disturbance = 0.0;
    double minimum_disturbed_speed = 10000.0;
    double recovered_speed = 0.0;
    double recovered_torque = 0.0;
    double reversed_speed = 0.0;
    double maximum_stop_speed = 0.0;

    printf("Scenario: noise %s\n", noise_enabled ? "on" : "off");
    simulation_init(&simulation, noise_enabled);
    for (i = 0; i < SIMULATION_TICKS; ++i) {
        sample = simulation_step(&simulation);
        if (!isfinite(sample.true_rpm) || !isfinite(sample.filtered_rpm)
            || !isfinite(sample.motor_torque_nm)
            || fabs(sample.motor_torque_nm) > MOTOR_TORQUE_LIMIT_NM + 1e-12) {
            finite_and_bounded = 0;
        }
        if (!near(sample.time_s, (double)i / 20.0, 1e-10)) {
            regular_timing = 0;
        }
        hit_saturation |= sample.saturated;
        if (i == 29 * CONTROL_HZ) {
            speed_before_disturbance = sample.true_rpm;
            torque_before_disturbance = sample.motor_torque_nm;
        }
        if (i >= 30 * CONTROL_HZ && i < 40 * CONTROL_HZ
            && sample.true_rpm < minimum_disturbed_speed) {
            minimum_disturbed_speed = sample.true_rpm;
        }
        if (i == 59 * CONTROL_HZ) {
            recovered_speed = sample.true_rpm;
            recovered_torque = sample.motor_torque_nm;
        }
        if (i == 94 * CONTROL_HZ) {
            reversed_speed = sample.true_rpm;
        }
        if (i >= 115 * CONTROL_HZ && fabs(sample.true_rpm) > maximum_stop_speed) {
            maximum_stop_speed = fabs(sample.true_rpm);
        }
    }
    check(simulation.tick == 2400 && regular_timing, "120 seconds contain exactly 2400 control updates at 20 Hz");
    check(finite_and_bounded, "all samples are finite and motor torque stays within limits");
    check(hit_saturation, "large speed step exercises output saturation");
    check(near(speed_before_disturbance, 1000.0, 1.0), "startup settles within 1 rpm before disturbance");
    check(minimum_disturbed_speed < 999.0, "increased friction causes a visible transient speed drop");
    check(near(recovered_speed, 1000.0, 1.0), "PI restores speed after friction increase");
    check(near(recovered_torque - torque_before_disturbance, 0.006, 0.0005),
          "steady motor torque rises by approximately 0.006 Nm");
    check(near(reversed_speed, -500.0, 1.0), "controller reaches negative speed after command reversal");
    check(maximum_stop_speed < 1.0, "last five seconds remain within 1 rpm of rest");
    printf("Evidence: pre=%.6f rpm, dip=%.6f rpm, recovered=%.6f rpm, "
           "torque_delta=%.9f Nm, reverse=%.6f rpm, stop_peak=%.6f rpm\n",
           speed_before_disturbance, minimum_disturbed_speed, recovered_speed,
           recovered_torque - torque_before_disturbance, reversed_speed, maximum_stop_speed);
}

int main(void)
{
    check(simulation_target_rpm(39) == 0.0 && simulation_target_rpm(40) == 1000.0,
          "startup command changes exactly at 2 seconds");
    check(simulation_target_rpm(1199) == 1000.0 && simulation_target_rpm(1200) == -500.0,
          "reversal command changes exactly at 60 seconds");
    check(simulation_target_rpm(1899) == -500.0 && simulation_target_rpm(1900) == 0.0,
          "stop command changes exactly at 95 seconds");
    test_moving_average();
    test_pi();
    test_wheel_model();
    test_scenario(0);
    test_scenario(1);
    printf("%d checks, %d failures\n", checks, failures);
    return failures == 0 ? 0 : 1;
}
