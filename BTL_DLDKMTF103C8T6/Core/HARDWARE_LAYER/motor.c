#include <motor.h>
#include"stm32f1xx_hal_conf.h"
extern TIM_HandleTypeDef htim1;
extern TIM_HandleTypeDef htim2;
void init_motor(motor_name* motor, GPIO_TypeDef* motor_ENA,uint16_t motor_ENA_pin,
				GPIO_TypeDef* motor_ENB,uint16_t motor_ENB_pin,
				uint32_t Encoder_A_channel,
				uint32_t Encoder_B_channel,
				uint32_t PWM_channel){
	motor->motor_ENA = motor_ENA;
	motor->motor_ENB = motor_ENB;
	motor->motor_ENA_pin = motor_ENA_pin;
	motor->motor_ENB_pin = motor_ENB_pin;
	motor->Encoder_A_channel = Encoder_A_channel;
	motor->Encoder_B_channel = Encoder_B_channel;
	motor->PWM_channel = PWM_channel;
	HAL_TIM_Encoder_Start(&htim1,motor->Encoder_A_channel | motor->Encoder_B_channel);
	HAL_TIM_PWM_Start(&htim2,motor->PWM_channel);
}
void motor_run_CW(motor_name* motor){
	HAL_GPIO_WritePin(GPIOB,GPIO_PIN_14,0);
	HAL_GPIO_WritePin(GPIOB,GPIO_PIN_15,1);
}
void motor_run_CCW(motor_name* motor){
	HAL_GPIO_WritePin(GPIOB,GPIO_PIN_14,1);
	HAL_GPIO_WritePin(GPIOB,GPIO_PIN_15,0);
}
void motor_stop(motor_name* motor){
	HAL_GPIO_WritePin(GPIOB,GPIO_PIN_14,0);
	HAL_GPIO_WritePin(GPIOB,GPIO_PIN_15,0);
}
