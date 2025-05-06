#include <setting_PID.h>
#include "reset_data.h"
extern TIM_HandleTypeDef htim2;
void PID_INIT_PARAM(motor_name* motor,float setpoint_val,double kp,double ki,double kd){
	motor->setpoint_val = setpoint_val;
	motor->kp = kp;
	motor->ki = ki;
	motor->kd = kd;
	reset_data(motor);
}
