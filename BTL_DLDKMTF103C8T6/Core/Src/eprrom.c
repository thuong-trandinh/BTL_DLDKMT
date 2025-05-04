/*
 * eprrom.c
 *
 *  Created on: May 3, 2025
 *      Author: HP
 */
#include "eprrom.h"

void EEPRROM_WriteBuffer(I2C_HandleTypeDef *hi2c,uint16_t start_addr,uint8_t *buf,uint8_t length){
	HAL_I2C_Mem_Write(hi2c, 0xA0, start_addr, length, buf, 8, 20);
}
void EEPRROM_ReadBuffer(I2C_HandleTypeDef *hi2c,uint16_t start_addr,uint8_t *buf,uint8_t length){
	HAL_I2C_Mem_Read(hi2c, 0xA1, start_addr, length, buf, 8, 20);
}



