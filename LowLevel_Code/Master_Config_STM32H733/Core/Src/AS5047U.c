/*
 * AS5047U.c
 *
 *  Created on: Jan 24, 2022
 *      Author: Thansak Pongpaket
 */

#include "AS5047U.h"

float EncPulse2Rad_Read(AS5047U *Enc,uint8_t inv_dir){
	AS5047U_Position_Highspeed_Read(Enc,inv_dir);
	return (Enc->Position * 0.000383495f) - Enc->Offset;
//	return (Enc->Position * 0.000383495f);
}

void AS5047U_init(AS5047U *dev, SPI_HandleTypeDef *hspiHandle,
		GPIO_TypeDef *CSGPIOTypedef, CRC_HandleTypeDef *hcrcHandle,
		uint16_t CSGPIOPin,float offset) {
	dev->hspiHandle = hspiHandle;
	dev->hcrcHandle = hcrcHandle;
	dev->CSGPIOTypedef = CSGPIOTypedef;
	dev->CSGPIOPin = CSGPIOPin;
	HAL_GPIO_WritePin(CSGPIOTypedef, CSGPIOPin, 1);
	dev->Error_Status.CORDIC_Overflow = 0;
	dev->Error_Status.Offset_Compensation_Not_Finished = 0;
	dev->Error_Status.Watchdog_Error = 0;
	dev->Error_Status.CRC_Error = 0;
	dev->Error_Status.Command_Error = 0;
	dev->Error_Status.Framing_Error = 0;
	dev->Error_Status.P2ram_Error = 0;
	dev->Error_Status.P2ram_Warning = 0;
	dev->Error_Status.MagHalf = 0;
	dev->Error_Status.Agc_warning = 0;

	/* User Variable */
	dev->Offset = offset * 0.000383495f;
}


/*
 * Data Frame Format
 *
 * for 24 bits Data frame
 * 23	x			(Do not Care -> 0)
 * 22	R/W			(0 for read , 1 for write)
 * 21:8 ADDR[13:0]
 * 7:0	CRC-8
 *
 *
 * for 16 bits Data frame high throughput
 *
 * 15 	x			(Do not Care -> 0)
 * 14 	R/W			(0 for read , 1 for write)
 * 13:0 ADDR[13:0]
 */




/*
 * This function for Non-Volatile Registers (OTP) Only
 */

inline void AS5047U_Write(AS5047U *dev,uint16_t Register_Address, uint16_t Data){
	uint8_t Buffer[3] = {};
	Buffer[0] = ((uint8_t) (Register_Address >> 8)) & 0xBF;
	Buffer[1] = (uint8_t) (Register_Address & 0xFF);
	Buffer[2] = (uint8_t) HAL_CRC_Calculate(dev->hcrcHandle, (uint32_t *)Buffer, 2) ^ 0xFF;
	HAL_GPIO_WritePin(dev->CSGPIOTypedef, dev->CSGPIOPin, 0);
	HAL_SPI_Transmit(dev->hspiHandle, Buffer, 3, 1);
	HAL_GPIO_WritePin(dev->CSGPIOTypedef, dev->CSGPIOPin, 1);

	for (uint16_t i=0; i <= 550 ; i++);  			//delay before sent data (#Base clock 550MHz)
	Buffer[0] = (uint8_t) (Data >> 8);
	Buffer[1] = (uint8_t) (Data & 0xFF);
	Buffer[2] = (uint8_t) HAL_CRC_Calculate(dev->hcrcHandle, (uint32_t *)Buffer, 2) ^ 0xFF;
	HAL_GPIO_WritePin(dev->CSGPIOTypedef, dev->CSGPIOPin, 0);
	HAL_SPI_Transmit(dev->hspiHandle, Buffer, 3, 1);
	HAL_GPIO_WritePin(dev->CSGPIOTypedef, dev->CSGPIOPin, 1);
}

inline HAL_StatusTypeDef AS5047U_Read(AS5047U *dev,uint16_t Register_Address, uint16_t *Data){
	uint8_t Buffer[3] = {};

	Buffer[0] = ((uint8_t) (Register_Address >> 8)) | 0x40;
	Buffer[1] = (uint8_t) (Register_Address & 0xFF);
	Buffer[2] = (uint8_t) HAL_CRC_Calculate(dev->hcrcHandle, (uint32_t *)Buffer, 2) ^ 0xFF;
	HAL_GPIO_WritePin(dev->CSGPIOTypedef, dev->CSGPIOPin, 0);
	HAL_SPI_Transmit(dev->hspiHandle, Buffer, 3, 1);
	HAL_GPIO_WritePin(dev->CSGPIOTypedef, dev->CSGPIOPin, 1);

	for (uint16_t i=0; i <= 550 ; i++);  			//delay before sent data (#Base clock 550MHz)
	HAL_GPIO_WritePin(dev->CSGPIOTypedef, dev->CSGPIOPin, 0);
	HAL_SPI_Receive(dev->hspiHandle, Buffer, 3, 1);
	HAL_GPIO_WritePin(dev->CSGPIOTypedef, dev->CSGPIOPin, 1);
	uint8_t AS5047U_crc = (uint8_t) HAL_CRC_Calculate(dev->hcrcHandle, (uint32_t*)Buffer, 2) ^ 0xFF;
	if (AS5047U_crc == Buffer[2]){
		return HAL_OK;
	}
	else{
		return HAL_ERROR;
	}
}


/*
 * This function for read Encoder without CRC
 * (high throughput)
 */
inline uint16_t AS5047U_Position_Highspeed_Read(AS5047U *dev,uint8_t dir){
	uint8_t cmd[2] = { 0x3F,0xFF };
	uint8_t Buffer[2] = {};
	for (uint16_t i=0; i <= 400; i++);
	HAL_GPIO_WritePin(dev->CSGPIOTypedef, dev->CSGPIOPin, 0);
	for (uint16_t i=0; i <= 400; i++);
	HAL_SPI_Transmit(dev->hspiHandle, cmd, 2, 100);
	for (uint16_t i=0; i <= 400; i++);
	HAL_GPIO_WritePin(dev->CSGPIOTypedef, dev->CSGPIOPin, 1);

	for (uint16_t i=0; i <= 550; i++);			//delay before sent data (#Base clock 550MHz)
	HAL_GPIO_WritePin(dev->CSGPIOTypedef, dev->CSGPIOPin, 0);
	for (uint16_t i=0; i <= 400; i++);
	HAL_SPI_Receive(dev->hspiHandle, Buffer, 2, 100);
	for (uint16_t i=0; i <= 400; i++);
	HAL_GPIO_WritePin(dev->CSGPIOTypedef, dev->CSGPIOPin, 1);
	if (dir == 1){
		dev->Position = (uint16_t)((((uint16_t)Buffer[0]&0x3F) << 8) | (uint16_t)Buffer[1]) ^ 0x3FFF;
	}
	else{
		dev->Position = ((((uint16_t)Buffer[0]&0x3F) << 8) | (uint16_t)Buffer[1]);
	}
	return dev->Position;
}

inline int16_t AS5047U_Speed_Highspeed_Read(AS5047U *dev) {
	uint8_t cmd[2] = { 0x3F, 0xFF };
	int16_t Output_Data;
	uint8_t Buffer[2] = { };
	HAL_GPIO_WritePin(dev->CSGPIOTypedef, dev->CSGPIOPin, 0);
	HAL_SPI_Transmit(dev->hspiHandle, cmd, 2, 1);
		HAL_GPIO_WritePin(dev->CSGPIOTypedef, dev->CSGPIOPin, 1);

	for (uint16_t i = 0; i <= 550; i++);		//delay before sent data (#Base clock 550MHz)
	HAL_GPIO_WritePin(dev->CSGPIOTypedef, dev->CSGPIOPin, 0);
	HAL_SPI_Receive(dev->hspiHandle, Buffer, 2, 1);
	HAL_GPIO_WritePin(dev->CSGPIOTypedef, dev->CSGPIOPin, 1);

	if ((Buffer[0] >> 5) == 0) {
		Output_Data = (((int16_t) Buffer[0] & 0x3F) << 8) | (int16_t) Buffer[1];
	} else if ((Buffer[0] >> 5) == 1) {
		Output_Data = (((int16_t) Buffer[0] | 0xC0) << 8) | (int16_t) Buffer[1];
	}
	return Output_Data;
}

void AS5047U_Read_Error(AS5047U *dev) {
	uint16_t Status;
	uint8_t cmd[2] = { 0x00,0x01 };
	uint8_t Buffer[2] = {};
	HAL_GPIO_WritePin(dev->CSGPIOTypedef, dev->CSGPIOPin, 0);
	HAL_SPI_Transmit(dev->hspiHandle, cmd, 2, 1);
	HAL_GPIO_WritePin(dev->CSGPIOTypedef, dev->CSGPIOPin, 1);
	for (uint16_t i=0; i <= 550 ; i++);			//delay before sent data (#Base clock 550MHz)
	HAL_GPIO_WritePin(dev->CSGPIOTypedef, dev->CSGPIOPin, 0);
	HAL_SPI_Receive(dev->hspiHandle, Buffer, 2, 1);
	HAL_GPIO_WritePin(dev->CSGPIOTypedef, dev->CSGPIOPin, 1);
	Status = (((uint16_t)Buffer[0]&0x3F) << 8) | (uint16_t)Buffer[1];

	AS5047U_Read(dev, 0x0001, &Status);
	dev->Error_Status.CORDIC_Overflow 					= (uint8_t) ((Status >> 10) & 0x01);
	dev->Error_Status.Offset_Compensation_Not_Finished 	= (uint8_t) ((Status >> 9) & 0x01);
	dev->Error_Status.Watchdog_Error 					= (uint8_t) ((Status >> 7) & 0x01);
	dev->Error_Status.CRC_Error 						= (uint8_t) ((Status >> 6) & 0x01);
	dev->Error_Status.Command_Error 					= (uint8_t) ((Status >> 5) & 0x01);
	dev->Error_Status.Framing_Error 					= (uint8_t) ((Status >> 4) & 0x01);
	dev->Error_Status.P2ram_Error 						= (uint8_t) ((Status >> 3) & 0x01);
	dev->Error_Status.P2ram_Warning 					= (uint8_t) ((Status >> 2) & 0x01);
	dev->Error_Status.MagHalf 							= (uint8_t) ((Status >> 1) & 0x01);
	dev->Error_Status.Agc_warning 						= (uint8_t) (Status & 0x01);
}
