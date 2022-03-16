/*
 * TPM75.h
 *
 *  Created on: Feb 3, 2022
 *      Author: thans
 */

#ifndef INC_LIBRARYINC_TPM75_H_
#define INC_LIBRARYINC_TPM75_H_

// include stm32h7 driver
#include "stm32h7xx.h"

typedef struct {
	I2C_HandleTypeDef *i2cHandle;
	uint8_t address;
	uint8_t Resolution;
	int16_t RawTemp;
	float Temp;
} TPM75;

void TPM75_init(TPM75 *dev, I2C_HandleTypeDef *i2cHandle, uint8_t A2, uint8_t A1, uint8_t A0);
HAL_StatusTypeDef TPM75_TempRead(TPM75 *dev);
HAL_StatusTypeDef TPM75_T_Low(TPM75 *dev, uint16_t *Temp);
HAL_StatusTypeDef TPM75_T_High(TPM75 *dev, uint16_t *Temp);
HAL_StatusTypeDef TPM75_Resolution_Set(TPM75 *dev, uint8_t Resolution);
HAL_StatusTypeDef TPM75_Fault_Count(TPM75 *dev, uint8_t Fault_Count);
HAL_StatusTypeDef TPM75_ALERT_Polarity(TPM75 *dev, uint8_t ALERT_Polarity);
HAL_StatusTypeDef TPM75_Shutdown_Mode(TPM75 *dev, uint8_t Shutdown_Mode);
HAL_StatusTypeDef TPM75_One_Shot(TPM75 *dev, uint8_t One_Shot);
HAL_StatusTypeDef TPM75_Thermostat_Mode(TPM75 *dev, uint8_t Thermostat_Mode);


#endif /* INC_LIBRARYINC_TPM75_H_ */
