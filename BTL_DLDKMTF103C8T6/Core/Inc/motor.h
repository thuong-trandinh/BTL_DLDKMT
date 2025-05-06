#ifndef INC_MOTOR_H_
#define INC_MOTOR_H_
#include "stm32f1xx_hal.h"
typedef struct{
	//PID parameter
	double kp,ki,kd;
	float error_value,error_pre1,error_pre2;
	float output_pwm,last_output_pwm;
	int16_t tick_current,tick_pre;
	float setpoint_val;
	//motor pin
	uint32_t Encoder_A_channel;
	uint32_t Encoder_B_channel;
	uint32_t PWM_channel;
	GPIO_TypeDef* motor_ENA;
	uint16_t motor_ENA_pin;
	GPIO_TypeDef* motor_ENB;
	uint16_t motor_ENB_pin;
}motor_name;
void init_motor(motor_name* motor,GPIO_TypeDef* motor_ENA,uint16_t motor_ENA_pin,
				GPIO_TypeDef* motor_ENB,uint16_t motor_ENB_pin,
				uint32_t Encoder_A_channel,
				uint32_t Encoder_B_channel,
				uint32_t PWM_channel);
void motor_run_CW();
void motor_run_CCW();
void motor_stop();
#endif /* INC_MOTOR_H_ */




