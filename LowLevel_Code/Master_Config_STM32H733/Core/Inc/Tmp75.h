#ifndef TMP75C_H_
#define TMP75C_H_

#include "stm32h7xx.h"

/*=============================================
== 
===============================================*/
#define TMP75_ADDR	0x92
#define FTMP75_TEMP_AD_STEP		0.0625

/*=============================================
== 
===============================================*/
void Tmp75_Init(I2C_HandleTypeDef *hi2c);
void Write_Register(uint8_t register_pointer, uint16_t register_value, I2C_HandleTypeDef *hi2c);
void Read_Register(uint8_t register_pointer, uint8_t* receive_buffer, I2C_HandleTypeDef *hi2c);
void Read_TempCelsius(double* receive_buffer, I2C_HandleTypeDef *hi2c);
void Read_TempEeprom(uint16_t* receive_buffer, I2C_HandleTypeDef *hi2c);
void One_ShotTemp(I2C_HandleTypeDef *hi2c);

#endif
