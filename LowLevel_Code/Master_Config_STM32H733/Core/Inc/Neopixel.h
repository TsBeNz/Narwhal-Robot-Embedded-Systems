/*
 * Neopixel.h
 *
 *  Created on: Nov 22, 2021
 *      Author: thans
 */

#ifndef INC_NEOPIXEL_H_
#define INC_NEOPIXEL_H_

// include stm32h7 driver
#include "stm32h7xx.h"

#define NUM_LED 5

typedef struct {
	uint16_t pwmData[(NUM_LED*24)+42];
	uint8_t LED_Data[NUM_LED][3];
	uint32_t timer_Channel;
	uint16_t Pulse_Low;
	uint16_t Pulse_High;
	TIM_HandleTypeDef *htim;
	uint32_t TimeEffect;
}NeopixelParameter;

void Neopixel_Init(NeopixelParameter *Neopixel_led, TIM_HandleTypeDef *htim_in,	uint32_t Channel_in);
void Neopixel_Set(NeopixelParameter *Neopixel_led, uint8_t LEDnum, uint8_t Red,	uint8_t Green, uint8_t Blue);
void Neopixel_Sent(NeopixelParameter *Neopixel_led);
void Neopixel_Rainbow_Effect(NeopixelParameter *Neopixel_led);

#endif /* INC_NEOPIXEL_H_ */
