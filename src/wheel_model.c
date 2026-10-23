#include <math.h>
#include "wheel_model.h"

#define PI_VALUE 3.14159265358979323846

double rpm_to_rad_s(double rpm)
{
    return rpm * (2.0 * PI_VALUE / 60.0);
}

double rad_s_to_rpm(double rad_s)
{
    return rad_s * (60.0 / (2.0 * PI_VALUE));
}

void wheel_model_init(WheelModel *wheel, double inertia, double viscous, double coulomb)
{
    wheel->speed_rad_s = 0.0;
    wheel->inertia_kg_m2 = inertia;
    wheel->viscous_nm_per_rad_s = viscous;
    wheel->coulomb_nm = coulomb;
}

double wheel_model_friction_nm(const WheelModel *wheel, double motor_torque_nm)
{
    double dry_friction;
    if (wheel->speed_rad_s > 0.0) {
        dry_friction = wheel->coulomb_nm;
    } else if (wheel->speed_rad_s < 0.0) {
        dry_friction = -wheel->coulomb_nm;
    } else if (fabs(motor_torque_nm) <= wheel->coulomb_nm) {
        /* 静止且驱动力矩不足时，静摩擦与驱动力矩平衡。 */
        dry_friction = motor_torque_nm;
    } else {
        dry_friction = motor_torque_nm > 0.0 ? wheel->coulomb_nm : -wheel->coulomb_nm;
    }
    return wheel->viscous_nm_per_rad_s * wheel->speed_rad_s + dry_friction;
}

void wheel_model_step(WheelModel *wheel, double motor_torque_nm, double dt_s)
{
    double old_speed = wheel->speed_rad_s;
    double friction = wheel_model_friction_nm(wheel, motor_torque_nm);
    double acceleration = (motor_torque_nm - friction) / wheel->inertia_kg_m2;
    double new_speed = old_speed + acceleration * dt_s;
    int crosses_zero = (old_speed > 0.0 && new_speed < 0.0)
                    || (old_speed < 0.0 && new_speed > 0.0);

    /* 不允许仅靠干摩擦把转子反向加速；反向驱动足够大时允许穿零。 */
    if (crosses_zero && fabs(motor_torque_nm) <= wheel->coulomb_nm) {
        new_speed = 0.0;
    }
    wheel->speed_rad_s = new_speed;
}
