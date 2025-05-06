#include <motor.h>
#include <process_read_tick.h>
#include "main.h"

#define T 0.01f

float calculate_current_angle(motor_name* motor){
	float current_angle = 0;
	static int16_t tick_current = 0;
	tick_current = read_current_tick_encoder(motor);
	current_angle = (tick_current * 360.0) / 1600;
	return current_angle;
}
float calculate_current_speed(motor_name* motor){
	float current_speed = 0;
	static int16_t tick_current = 0;
	tick_current = read_current_tick_encoder(motor);
	float tick_cnt = tick_current - motor->tick_pre; // ticks per 10 ms
	current_speed = (tick_cnt * 60.0)/(1600 * T);
	motor->tick_pre = tick_current;
	return current_speed;
}
