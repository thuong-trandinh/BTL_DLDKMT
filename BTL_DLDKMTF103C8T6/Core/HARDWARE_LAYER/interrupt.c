#include "interrupt.h"
extern TIM_HandleTypeDef htim3;
void init_interrupt(TIM_HandleTypeDef* htim){
	HAL_TIM_Base_Start_IT(&htim3);
}
