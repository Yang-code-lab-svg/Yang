#ifndef SIMULATION_H
#define SIMULATION_H

#include "moving_average.h"
#include "pi_controller.h"
#include "wheel_model.h"

typedef struct {
    int tick;
    int noise_enabled;
    MovingAverage filter;
    PiController controller;
    WheelModel wheel;
} Simulation;

typedef struct {
    double time_s;
    double target_rpm;
    double true_rpm;
    double measured_rpm;
    double filtered_rpm;
    double motor_torque_nm;
    double friction_torque_nm;
    double integral_nm;
    int saturated;
} SimulationSample;

void simulation_init(Simulation *simulation, int noise_enabled);
double simulation_target_rpm(int tick);
/* 每次调用推进50 ms仿真时间；返回本区间起点的状态与控制量。 */
SimulationSample simulation_step(Simulation *simulation);

#endif
