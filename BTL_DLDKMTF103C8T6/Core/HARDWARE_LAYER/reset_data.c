#include"reset_data.h"
extern TIM_HandleTypeDef htim2;
void reset_data(motor_name* motor){
		TIM1->CNT = 0;
		motor->error_pre1 = 0;
		motor->error_pre2 = 0;
		motor->output_pwm = 0;
		motor->tick_current = 0;
		motor->tick_pre = 0;
		motor->error_value = 0;
		motor->last_output_pwm = 0;
		__HAL_TIM_SET_COMPARE(&htim2,TIM_CHANNEL_1,0);
}
