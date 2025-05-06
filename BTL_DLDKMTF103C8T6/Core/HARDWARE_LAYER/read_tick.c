#include <read_tick.h>
extern TIM_HandleTypeDef htim1;
uint16_t read_current_tick_encoder(motor_name* motor){
	motor->tick_current = __HAL_TIM_GET_COUNTER(&htim1);
	return motor->tick_current;
}
