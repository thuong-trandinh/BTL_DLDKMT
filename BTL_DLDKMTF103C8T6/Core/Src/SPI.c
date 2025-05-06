/*
 * SPI.c
 *
 *  Created on: Apr 6, 2025
 *      Author: HP
 */

#include <SPI.h>
extern SPI_HandleTypeDef hspi1;
void send_data(uint8_t address,uint8_t value)
{
	uint8_t data[2];
	data[0] = address&0x0F;
	data[1] = value;
	HAL_GPIO_WritePin(GPIOA, GPIO_PIN_4, 0);
	HAL_SPI_Transmit(&hspi1, data, 2, 10);
	HAL_GPIO_WritePin(GPIOA, GPIO_PIN_4, 1);
}
void MAX7219_Init()
{

	send_data(0x09,0xFF); // Decode Mode Init
	send_data(0x0A,0x03); // Intensity
	send_data(0x0B,0x07); // Scan - Limit
	send_data(0x0C,0x01); // Shutdown register
	send_data(0x0F, 0x00);

}
void Send_Value(uint8_t *value){
	for(uint8_t i = 0;i < 8; i ++){
		send_data(8 - i,*(value+i));
	}
}

