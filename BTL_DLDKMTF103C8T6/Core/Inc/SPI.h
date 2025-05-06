/*
 * SPI.h
 *
 *  Created on: Apr 6, 2025
 *      Author: HP
 */

#ifndef INC_SPI_H_
#define INC_SPI_H_
#include "stm32f1xx_hal.h"
#include "stm32f1xx_hal_spi.h"

void send_data(uint8_t value,uint8_t address);
void MAX7219_Init();
void Send_Value(uint8_t *value);
#endif /* INC_SPI_H_ */
