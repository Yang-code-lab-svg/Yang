#ifndef PI_CONTROLLER_H
#define PI_CONTROLLER_H

typedef struct {
    double kp;
    double ki;
    double dt_s;
    double output_min_nm;
    double output_max_nm;
    double integral_nm;
    int saturated;
} PiController;

/* 约定：kp、ki >= 0，dt_s > 0，output_min_nm < 0 < output_max_nm。 */
void pi_init(PiController *pi, double kp, double ki, double dt_s,
             double output_min_nm, double output_max_nm);
double pi_update(PiController *pi, double target_rpm, double feedback_rpm);

#endif
