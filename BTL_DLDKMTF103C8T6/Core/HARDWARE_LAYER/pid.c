#include"PID.h"
float P_part = 0;
float I_part = 0;
float D_part = 0;
float saturation(motor_name* motor){
	if(motor->output_pwm > 254){
			 motor->output_pwm = 254;
	}
	if(motor->output_pwm < -254){
			motor->output_pwm = -254;
	}
	return motor->output_pwm;
}
float PID_Calculation(motor_name* motor,float setpoint_value,float current_value){
	//establish formula for PID parameter
	motor->setpoint_val = setpoint_value;
	motor->error_value = motor->setpoint_val - current_value;
	P_part = motor->kp*(motor->error_value - motor->error_pre1);
	I_part = 0.5*motor->ki*T*(motor->error_value + motor->error_pre1);
	D_part = motor->kd *(motor->error_value - 2*motor->error_pre1 + motor->error_pre2) / T;
	motor->output_pwm = motor->last_output_pwm + P_part + I_part + D_part;
	//store data
	motor->last_output_pwm = motor->output_pwm;
	motor->error_pre1 = motor->error_value;
	motor->error_pre2 = motor->error_pre1;
	//process raw output data and export final output control value
	float final_output = saturation(motor);
	return final_output;
}


