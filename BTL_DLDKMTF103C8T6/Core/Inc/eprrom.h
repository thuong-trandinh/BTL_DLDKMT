/*
 * eprrom.h
 *
 *  Created on: May 3, 2025
 *      Author: HP
 */

#ifndef INC_EPRROM_H_
#define INC_EPRROM_H_

#include <stm32f1xx.h>
#include "stm32f1xx_hal_i2c.h"

void EEPRROM_WriteBuffer(I2C_HandleTypeDef *hi2c,uint16_t start_addr,uint8_t *buf,uint8_t length);
void EEPRROM_ReadBuffer(I2C_HandleTypeDef *hi2c,uint16_t start_addr,uint8_t *buf,uint8_t length);


#endif /* INC_EPRROM_H_ */
