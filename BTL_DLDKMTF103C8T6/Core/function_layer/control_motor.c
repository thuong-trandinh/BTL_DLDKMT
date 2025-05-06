#include <motor.h>
#include"control_motor.h"
#include "PID.h"
extern TIM_HandleTypeDef htim2;
void Xuat_PWM_2(float Duty_Cycle)
{
	Duty_Cycle = Duty_Cycle*__HAL_TIM_GET_AUTORELOAD(&htim2)/100;
	__HAL_TIM_SET_COMPARE(&htim2,TIM_CHANNEL_1,Duty_Cycle);
}
void control_motor_status(motor_name* motor,float duty_cycle){
	if(motor->output_pwm > 0){
			motor_run_CCW(); // positive tick
			__HAL_TIM_SET_COMPARE(&htim2,TIM_CHANNEL_1,duty_cycle);

	}
	else if(motor->output_pwm < 0){
			motor_run_CW(); // negative tick
			__HAL_TIM_SET_COMPARE(&htim2,TIM_CHANNEL_1,fabs(duty_cycle));
	}
	else{
			motor_stop();
			__HAL_TIM_SET_COMPARE(&htim2,TIM_CHANNEL_1,0);
	}
}

