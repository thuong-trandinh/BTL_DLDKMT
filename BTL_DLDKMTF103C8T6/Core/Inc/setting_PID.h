#ifndef INC_SETTING_PID_H_
#define INC_SETTING_PID_H_
#include <read_tick.h>
#include <motor.h>
void PID_INIT_PARAM(motor_name* motor,float setpoint_val,double kp,double ki,double kd);
#endif /* INC_SETTING_PID_H_ */
