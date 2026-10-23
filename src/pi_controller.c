#include "pi_controller.h"

static double clamp(double value, double minimum, double maximum)
{
    if (value > maximum) {
        return maximum;
    }
    if (value < minimum) {
        return minimum;
    }
    return value;
}

void pi_init(PiController *pi, double kp, double ki, double dt_s,
             double output_min_nm, double output_max_nm)
{
    pi->kp = kp;
    pi->ki = ki;
    pi->dt_s = dt_s;
    pi->output_min_nm = output_min_nm;
    pi->output_max_nm = output_max_nm;
    pi->integral_nm = 0.0;
    pi->saturated = 0;
}

double pi_update(PiController *pi, double target_rpm, double feedback_rpm)
{
    double error_rpm = target_rpm - feedback_rpm;
    double proportional_nm = pi->kp * error_rpm;
    double candidate_integral = pi->integral_nm + pi->ki * error_rpm * pi->dt_s;
    double candidate_output = proportional_nm + candidate_integral;
    double output_nm;
    int pushes_upper_limit = candidate_output > pi->output_max_nm && error_rpm > 0.0;
    int pushes_lower_limit = candidate_output < pi->output_min_nm && error_rpm < 0.0;

    /* 条件积分抗饱和：输出越限且误差继续推向该限值时，冻结积分。 */
    if (!pushes_upper_limit && !pushes_lower_limit) {
        pi->integral_nm = clamp(candidate_integral,
                                pi->output_min_nm, pi->output_max_nm);
    }
    output_nm = proportional_nm + pi->integral_nm;
    pi->saturated = output_nm >= pi->output_max_nm || output_nm <= pi->output_min_nm;
    return clamp(output_nm, pi->output_min_nm, pi->output_max_nm);
}
