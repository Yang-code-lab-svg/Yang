#include <math.h>
#include <stdio.h>
#include <string.h>
#include "config.h"
#include "simulation.h"

static void print_usage(const char *program)
{
    printf("Usage: %s [--output FILE.csv] [--no-noise] [--help]\n", program);
    puts("Runs 120 seconds of synthetic simulation time (20 Hz control).");
}

int main(int argc, char *argv[])
{
    const char *output_path = "simulation.csv";
    int noise_enabled = 1;
    int i;
    int saturated_ticks = 0;
    double peak_torque = 0.0;
    FILE *csv;
    Simulation simulation;

    for (i = 1; i < argc; ++i) {
        if (strcmp(argv[i], "--no-noise") == 0) {
            noise_enabled = 0;
        } else if (strcmp(argv[i], "--output") == 0 && i + 1 < argc) {
            output_path = argv[++i];
            if (output_path[0] == '\0' || output_path[0] == '-') {
                fputs("Error: --output requires a file path.\n", stderr);
                return 2;
            }
        } else if (strcmp(argv[i], "--help") == 0) {
            print_usage(argv[0]);
            return 0;
        } else {
            fprintf(stderr, "Error: unknown or incomplete argument: %s\n", argv[i]);
            print_usage(argv[0]);
            return 2;
        }
    }

    csv = fopen(output_path, "w");
    if (csv == NULL) {
        perror("Cannot open CSV output");
        return 1;
    }
    if (fputs("time_s,target_rpm,true_rpm,measured_rpm,filtered_rpm,motor_torque_nm,"
              "friction_torque_nm,integral_nm,saturated\n", csv) == EOF) {
        perror("Cannot write CSV header");
        fclose(csv);
        return 1;
    }

    simulation_init(&simulation, noise_enabled);
    puts("Reaction wheel speed-control simulation (synthetic data only)");
    printf("Control: %d Hz; dt: %.3f s; filter: %d samples; noise: %s\n",
           CONTROL_HZ, CONTROL_DT_S, FILTER_WINDOW, noise_enabled ? "on" : "off");
    puts(" time_s  target_rpm    true_rpm  filtered_rpm  motor_torque_Nm");
    for (i = 0; i < SIMULATION_TICKS; ++i) {
        SimulationSample sample = simulation_step(&simulation);
        if (!isfinite(sample.true_rpm) || !isfinite(sample.motor_torque_nm)) {
            fputs("Error: non-finite simulation state.\n", stderr);
            fclose(csv);
            return 1;
        }
        if (fprintf(csv, "%.3f,%.6f,%.6f,%.6f,%.6f,%.9f,%.9f,%.9f,%d\n",
                    sample.time_s, sample.target_rpm, sample.true_rpm,
                    sample.measured_rpm, sample.filtered_rpm, sample.motor_torque_nm,
                    sample.friction_torque_nm, sample.integral_nm, sample.saturated) < 0) {
            perror("Cannot write CSV sample");
            fclose(csv);
            return 1;
        }
        if (i % (10 * CONTROL_HZ) == 0) {
            printf("%7.2f %11.2f %11.3f %13.3f %16.6f\n", sample.time_s,
                   sample.target_rpm, sample.true_rpm, sample.filtered_rpm,
                   sample.motor_torque_nm);
        }
        saturated_ticks += sample.saturated;
        if (fabs(sample.motor_torque_nm) > peak_torque) {
            peak_torque = fabs(sample.motor_torque_nm);
        }
    }
    if (fclose(csv) != 0) {
        perror("Cannot close CSV output");
        return 1;
    }
    printf("Completed: %d ticks, %.2f s simulated; final speed: %.6f rpm\n",
           simulation.tick, (double)simulation.tick * CONTROL_DT_S,
           rad_s_to_rpm(simulation.wheel.speed_rad_s));
    printf("Peak |motor torque|: %.6f Nm; saturated ticks: %d\n", peak_torque, saturated_ticks);
    printf("CSV: %s\n", output_path);
    return 0;
}
