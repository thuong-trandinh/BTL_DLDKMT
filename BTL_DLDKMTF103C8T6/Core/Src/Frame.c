/*
 * Frame.c
 *
 *  Created on: Apr 13, 2025
 *      Author: HP
 */


#include "Frame.h"


uint16_t Calculate_CRC16(uint8_t *data, uint16_t length) {
    uint16_t crc = 0xFFFF;
    for (uint16_t i = 0; i < length; i++) {
        crc ^= (uint16_t)(data[i] << 8);  // Sử dụng data[i] cho rõ ràng
        for (uint8_t j = 0; j < 8; j++) {
            if (crc & 0x8000)
                crc = (crc << 1) ^ 0x1021;
            else
                crc = crc << 1;
        }
    }
    return crc;
}
void packframe(uint8_t *outbuf, frame_t  *trans,uint8_t *payload, uint8_t command){
	trans->start_bit=0xAA;
	trans->end = 0xAF;
	outbuf[0] = trans->start_bit;
	trans->command = command;
	outbuf[1] = trans->command;
	outbuf[2] = (uint8_t)(trans->length >> 8)&0xFF; // MSB
	outbuf[3] = (uint8_t)(trans->length & 0xFF);
	// Copy payload vào frame (nếu có)
	for(uint8_t i = 0;i < trans->length;i++){
		trans->payload[i] = payload[i];
	}
	if (trans->length > 0) {
	    memcpy(&outbuf[4], trans->payload, trans->length);
	}
	trans->CRC_16 = Calculate_CRC16(&outbuf[1],trans->length + 3); // temp ktr xiu nua xoa
	outbuf[trans->length + 4] = ((trans->CRC_16)>>8)&0xFF;
	outbuf[trans->length + 5] = trans->CRC_16&0xFF;
	outbuf[6 + trans->length] = trans->end;
}
void Decode_frame(uint8_t *outbuf, frame_t *trans){
	trans->start_bit = outbuf[0];
	trans->command = outbuf[1];
	trans->length = (outbuf[2]<<8)|outbuf[3];
	trans->CRC_16 = (outbuf[4 + trans->length]<<8)|outbuf[5 + trans->length];
	trans->end = outbuf[6 + trans->length];
	for(int i = 0;i < trans->length;i++){
		trans->payload[i] = outbuf[i + 4];
	}
}
void Decode_Payload(uint8_t *outbuf, frame_t *trans,uint16_t *addr,uint8_t *bytes){
	*addr = (trans->payload[0]<<8)|(trans->payload[1]);
	if(*bytes <= 0){
		*bytes = trans->payload[2];
		return;
	}else{
		*bytes = trans->payload[2];
		for(uint8_t i = 0;i < *bytes; i++){
			outbuf[i] = trans->payload[i + 3];
		}
	}
}
void Decode_Position(float *outbuf, frame_t *trans){
	for(int i = 0; i < trans->length / 4; i++){
		 // ghép 4 byte MSB→LSB vào uint32_t
		 uint32_t u =  ((uint32_t)trans->payload[4*i    ] << 24)
		               | ((uint32_t)trans->payload[4*i + 1] << 16)
		               | ((uint32_t)trans->payload[4*i + 2] <<  8)
		               | ((uint32_t)trans->payload[4*i + 3] <<  0);
		 float f;
		 memcpy(&f, &u, sizeof(f));    // giữ nguyên bit‑pattern
		 outbuf[i] = f;
	}
}
void FloatToBytes(float value, uint8_t *out) {
    uint32_t asInt;
    /* Copy bit-pattern of float into 32-bit integer */
    memcpy(&asInt, &value, sizeof(asInt));
    /* Extract bytes in big-endian order (MSB first) */
    out[0] = (uint8_t)((asInt >> 24) & 0xFF);
    out[1] = (uint8_t)((asInt >> 16) & 0xFF);
    out[2] = (uint8_t)((asInt >> 8) & 0xFF);
    out[3] = (uint8_t)(asInt & 0xFF);
}


