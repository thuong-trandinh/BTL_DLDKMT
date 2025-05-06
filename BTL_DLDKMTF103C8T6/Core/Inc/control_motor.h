#ifndef INC_CONTROL_MOTOR_H_
#define INC_CONTROL_MOTOR_H_
#include <read_tick.h>
#include"math.h"
#include"stm32f1xx_hal.h"
void control_motor_status(motor_name* motor,float duty_cycle);
#endif /* INC_CONTROL_MOTOR_H_ */
