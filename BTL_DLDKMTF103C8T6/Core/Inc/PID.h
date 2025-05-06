#ifndef INC_PID_H_
#define INC_PID_H_
#include"motor.h"
#include"read_tick.h"
#define T 0.01
float Xuat_PWM(float Duty_Cycle);
float saturation(motor_name* motor);
float PID_Calculation(motor_name* motor,float setpoint_value,float current_value);
#endif /* INC_PID_H_ */
