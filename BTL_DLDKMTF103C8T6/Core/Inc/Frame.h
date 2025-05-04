/*
 * Frame.h
 *
 *  Created on: Apr 13, 2025
 *      Author: HP
 */

#ifndef INC_FRAME_H_
#define INC_FRAME_H_
#include "stm32f1xx.h"
#include <string.h>
#include <stdlib.h>

typedef struct
{
	uint8_t start_bit; // 1byte = 0xAA
	uint8_t command;
	uint16_t length;
	uint8_t payload[5000];
	uint16_t CRC_16;
	uint8_t end ; // 0xAF
} frame_t;

uint16_t Calculate_CRC16(uint8_t *data, uint16_t length);
void packframe(uint8_t *outbuf, frame_t  *trans,uint8_t *payload, uint8_t command);
void Decode_frame(uint8_t *outbuf, frame_t *trans);
void Decode_Payload(uint8_t *outbuf, frame_t *trans,uint16_t *addr,uint8_t *bytes);
#endif /* INC_FRAME_H_ */
