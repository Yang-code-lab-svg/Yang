#ifndef WHEEL_MODEL_H
#define WHEEL_MODEL_H

typedef struct {
    double speed_rad_s;
    double inertia_kg_m2;
    double viscous_nm_per_rad_s;
    double coulomb_nm;
} WheelModel;

double rpm_to_rad_s(double rpm);
double rad_s_to_rpm(double rad_s);
/* 约定：inertia > 0，viscous、coulomb >= 0，step 的 dt_s > 0。 */
void wheel_model_init(WheelModel *wheel, double inertia, double viscous, double coulomb);
double wheel_model_friction_nm(const WheelModel *wheel, double motor_torque_nm);
void wheel_model_step(WheelModel *wheel, double motor_torque_nm, double dt_s);

#endif
