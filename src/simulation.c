#include "config.h"
#include "simulation.h"

static double synthetic_noise_rpm(int tick)
{
    /* 固定的7拍序列：无需随机种子，各电脑上的测试可重复。 */
    static const double noise[] = {0.6, -0.4, 0.9, -0.8, 0.2, -0.6, 0.1};
    return noise[tick % 7];
}

void simulation_init(Simulation *simulation, int noise_enabled)
{
    simulation->tick = 0;
    simulation->noise_enabled = noise_enabled;
    moving_average_init(&simulation->filter);
    pi_init(&simulation->controller, PI_KP_NM_PER_RPM, PI_KI_NM_PER_RPM_S,
            CONTROL_DT_S, -MOTOR_TORQUE_LIMIT_NM, MOTOR_TORQUE_LIMIT_NM);
    wheel_model_init(&simulation->wheel, MODEL_INERTIA_KG_M2,
                     MODEL_VISCOUS_NM_PER_RAD_S, MODEL_INITIAL_COULOMB_NM);
}

double simulation_target_rpm(int tick)
{
    if (tick < 2 * CONTROL_HZ) {
        return 0.0;
    }
    if (tick < 60 * CONTROL_HZ) {
        return 1000.0;
    }
    if (tick < 95 * CONTROL_HZ) {
        return -500.0;
    }
    return 0.0;
}

SimulationSample simulation_step(Simulation *simulation)
{
    SimulationSample sample;
    int substep;
    if (simulation->tick >= 30 * CONTROL_HZ) {
        simulation->wheel.coulomb_nm = MODEL_INCREASED_COULOMB_NM;
    }

    /* 整数拍计时，避免反复累加0.05带来的事件边界漂移。 */
    sample.time_s = (double)simulation->tick * CONTROL_DT_S;
    sample.target_rpm = simulation_target_rpm(simulation->tick);
    sample.true_rpm = rad_s_to_rpm(simulation->wheel.speed_rad_s);
    sample.measured_rpm = sample.true_rpm;
    if (simulation->noise_enabled) {
        sample.measured_rpm += synthetic_noise_rpm(simulation->tick);
    }
    sample.filtered_rpm = moving_average_update(&simulation->filter, sample.measured_rpm);
    sample.motor_torque_nm = pi_update(&simulation->controller,
                                       sample.target_rpm, sample.filtered_rpm);
    sample.friction_torque_nm = wheel_model_friction_nm(&simulation->wheel,
                                                       sample.motor_torque_nm);
    sample.integral_nm = simulation->controller.integral_nm;
    sample.saturated = simulation->controller.saturated;

    /* 控制输出保持50 ms；仅模型以1 ms小步积分，不增加控制频率。 */
    for (substep = 0; substep < MODEL_SUBSTEPS; ++substep) {
        wheel_model_step(&simulation->wheel, sample.motor_torque_nm,
                         CONTROL_DT_S / (double)MODEL_SUBSTEPS);
    }
    ++simulation->tick;
    return sample;
}
