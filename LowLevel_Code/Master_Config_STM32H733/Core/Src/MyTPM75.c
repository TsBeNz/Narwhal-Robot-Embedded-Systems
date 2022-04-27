/*
 * TPM75.c
 *
 *  Created on: Feb 3, 2022
 *      Author: thansak Pongpraket
 */

#include <TPM75.h>

void TPM75_init(TPM75 *dev, I2C_HandleTypeDef *i2cHandle, uint8_t A2, uint8_t A1, uint8_t A0) {
	dev->address = 0x92;
//	dev->address = 0x92;
}


/*
 * Temperature Read
 */
HAL_StatusTypeDef TPM75_TempRead(TPM75 *dev) {
	uint8_t Buffer[2];
	HAL_StatusTypeDef Status_Buffer = HAL_I2C_Mem_Read(dev->i2cHandle, dev->address, 0x00,
	I2C_MEMADD_SIZE_8BIT, Buffer, 2, 100);
	dev->RawTemp = (uint16_t)Buffer[1] | (uint16_t)Buffer[0] << 8;
	dev->Temp = (double)(dev->RawTemp)*(dev->Resolution);
	return Status_Buffer;
}


/*
 *	Set Low Temperature Register
 */
HAL_StatusTypeDef TPM75_T_Low(TPM75 *dev, uint16_t *Temp) {
	uint8_t Buffer[2] = { (uint8_t) (0x00FF & (*Temp >> 8)), (uint8_t) (0x00FF & *Temp) };
	return HAL_I2C_Mem_Write(dev->i2cHandle, dev->address, 0x02,
	I2C_MEMADD_SIZE_8BIT, Buffer, 2, 100);
}


/*
 *	Set High Temperature Register
 */
HAL_StatusTypeDef TPM75_T_High(TPM75 *dev, uint16_t *Temp) {
	uint8_t Buffer[2] = { (uint8_t) (0x00FF & (*Temp >> 8)), (uint8_t) (0x00FF & *Temp) };
	return HAL_I2C_Mem_Write(dev->i2cHandle, dev->address, 0x03,
	I2C_MEMADD_SIZE_8BIT, Buffer, 2, 100);
}


/*
 * Resolution
 *
 * 0 = 9 bits (0.5°C) 		-> 27.5 ms
 * 1 = 10 bits (0.25°C)		-> 55 	ms
 * 2 = 11 bits (0.125°C)	-> 110	ms
 * 3 = 12 bits (0.0625°C)	-> 220	ms
 */
HAL_StatusTypeDef TPM75_Resolution_Set(TPM75 *dev, uint8_t Resolution) {
	uint8_t Buffer;
	dev->Resolution = Resolution;
	HAL_StatusTypeDef Status_Buffer = HAL_I2C_Mem_Read(dev->i2cHandle, dev->address, 0x01,
	I2C_MEMADD_SIZE_8BIT, &Buffer, 1, 100);
	if (Status_Buffer != HAL_OK){
		return Status_Buffer;
	}
	Buffer &= 0x9F;
	Buffer |= ((Resolution & 0x03) << 5);
	Status_Buffer =  HAL_I2C_Mem_Write(dev->i2cHandle, dev->address, 0x01,
	I2C_MEMADD_SIZE_8BIT, &Buffer, 1, 100);
	return Status_Buffer;
}


/*
 * CONSECUTIVE FAULTS
 *
 * 0 = 1 Count
 * 1 = 2 Count
 * 2 = 4 Count
 * 3 = 6 Count
 */
HAL_StatusTypeDef TPM75_Fault_Count(TPM75 *dev, uint8_t Fault_Count) {
	uint8_t Buffer;
	HAL_StatusTypeDef Status_Buffer = HAL_I2C_Mem_Read(dev->i2cHandle, dev->address, 0x01,
	I2C_MEMADD_SIZE_8BIT, &Buffer, 1, 100);
	if (Status_Buffer != HAL_OK){
		return Status_Buffer;
	}
	Buffer &= 0xE7;
	Buffer |= ((Fault_Count & 0x03) << 3);
	Status_Buffer =  HAL_I2C_Mem_Write(dev->i2cHandle, dev->address, 0x01,
	I2C_MEMADD_SIZE_8BIT, &Buffer, 1, 100);
	return Status_Buffer;
}


/*
 * ALERT Polarity
 *
 * 0 (default) 	-> Active Low
 * 1 			-> Active High
 */
HAL_StatusTypeDef TPM75_ALERT_Polarity(TPM75 *dev, uint8_t ALERT_Polarity) {
	uint8_t Buffer;
	HAL_StatusTypeDef Status_Buffer = HAL_I2C_Mem_Read(dev->i2cHandle, dev->address, 0x01,
	I2C_MEMADD_SIZE_8BIT, &Buffer, 1, 100);
	if (Status_Buffer != HAL_OK){
		return Status_Buffer;
	}
	Buffer &= 0xFB;
	Buffer |= ((ALERT_Polarity & 0x01) << 2);
	Status_Buffer =  HAL_I2C_Mem_Write(dev->i2cHandle, dev->address, 0x01,
	I2C_MEMADD_SIZE_8BIT, &Buffer, 1, 100);
	return Status_Buffer;
}


/*
 * Shutdown Mode
 *
 * 0 (default) 	-> the device maintains a continuous conversion
 * 1 			-> the device shuts down
 */
HAL_StatusTypeDef TPM75_Shutdown_Mode(TPM75 *dev, uint8_t Shutdown_Mode) {
	uint8_t Buffer;
	HAL_StatusTypeDef Status_Buffer = HAL_I2C_Mem_Read(dev->i2cHandle, dev->address, 0x01,
	I2C_MEMADD_SIZE_8BIT, &Buffer, 1, 100);
	if (Status_Buffer != HAL_OK){
		return Status_Buffer;
	}
	Buffer &= 0xFE;
	Buffer |= (Shutdown_Mode & 0x01);
	Status_Buffer =  HAL_I2C_Mem_Write(dev->i2cHandle, dev->address, 0x01,
	I2C_MEMADD_SIZE_8BIT, &Buffer, 1, 100);
	return Status_Buffer;
}

/*
 * One-Shot conversion
 *
 * 1 	-> Start One-Shot conversion
 */
HAL_StatusTypeDef TPM75_One_Shot(TPM75 *dev, uint8_t One_Shot) {
	uint8_t Buffer;
	HAL_StatusTypeDef Status_Buffer = HAL_I2C_Mem_Read(dev->i2cHandle, dev->address, 0x01,
	I2C_MEMADD_SIZE_8BIT, &Buffer, 1, 100);
	if (Status_Buffer != HAL_OK){
		return Status_Buffer;
	}
	Buffer &= 0xEF;
	Buffer |= ((One_Shot & 0x01) << 7);
	Status_Buffer =  HAL_I2C_Mem_Write(dev->i2cHandle, dev->address, 0x01,
	I2C_MEMADD_SIZE_8BIT, &Buffer, 1, 100);
	return Status_Buffer;
}

/*
 * Thermostat Mode
 * 0	-> Comparator Mode
 * 1 	-> Interrupt Mode
 */
HAL_StatusTypeDef TPM75_Thermostat_Mode(TPM75 *dev, uint8_t Thermostat_Mode) {
	uint8_t Buffer;
	HAL_StatusTypeDef Status_Buffer = HAL_I2C_Mem_Read(dev->i2cHandle, dev->address, 0x01,
	I2C_MEMADD_SIZE_8BIT, &Buffer, 1, 100);
	if (Status_Buffer != HAL_OK){
		return Status_Buffer;
	}
	Buffer &= 0xFD;
	Buffer |= ((Thermostat_Mode & 0x01) << 1);
	Status_Buffer =  HAL_I2C_Mem_Write(dev->i2cHandle, dev->address, 0x01,
	I2C_MEMADD_SIZE_8BIT, &Buffer, 1, 100);
	return Status_Buffer;
}
