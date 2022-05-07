/*
 * Neopixel.c
 *
 *  Created on: Nov 22, 2021
 *      Author: thans
 */

#include "Neopixel.h"

void Neopixel_Init(NeopixelParameter *Neopixel_led, TIM_HandleTypeDef *htim_in,
		uint32_t Channel_in) {
	Neopixel_led->htim = htim_in;
	Neopixel_led->timer_Channel = Channel_in;
	Neopixel_led->Pulse_High = (uint16_t) (htim_in->Init.Period * 0.66f);
	Neopixel_led->Pulse_Low = (uint16_t) (htim_in->Init.Period * 0.33f);
}

inline void Neopixel_Set(NeopixelParameter *Neopixel_led, uint8_t LEDnum,
		uint8_t Red, uint8_t Green, uint8_t Blue) {
	Neopixel_led->LED_Data[LEDnum][0] = Green;
	Neopixel_led->LED_Data[LEDnum][1] = Red;
	Neopixel_led->LED_Data[LEDnum][2] = Blue;
}

inline void Neopixel_Sent(NeopixelParameter *Neopixel_led) {
	uint32_t index = 0;
	uint32_t data;
	for (int j = 0; j < NUM_LED; j++) {
		data = ((Neopixel_led->LED_Data[j][0] & 0xFF) << 16)
				| ((Neopixel_led->LED_Data[j][1] & 0xFF) << 8)
				| (Neopixel_led->LED_Data[j][2] & 0xFF);
		for (int i = 23; i >= 0; i--) {
			if (data & (1 << i)) {
				Neopixel_led->pwmData[index] = Neopixel_led->Pulse_High;	//192
			} else {
				Neopixel_led->pwmData[index] = Neopixel_led->Pulse_Low;		//96
			}
			index++;
		}
	}
	for (int i = 0; i < 42; i++) {
		Neopixel_led->pwmData[index] = 0;
		index++;
	}
	HAL_TIM_PWM_Start_DMA(Neopixel_led->htim, TIM_CHANNEL_1,
			Neopixel_led->pwmData, index);
}

void Neopixel_Rainbow_Effect(NeopixelParameter *Neopixel_led){
	uint16_t RGBPositionInit = Neopixel_led->TimeEffect % 1536;
	uint8_t R,G,B;
	for (uint8_t i = 0; i < NUM_LED; i++) {
		uint16_t RGBPosition = RGBPositionInit % 1536;
		if (RGBPosition < 256) {
			R = 255;
			G = (RGBPosition & 0xFF);
			B = 0;
		} else if (RGBPosition < 512) {
			R = 255 - (RGBPosition & 0xFF);
			G = 255;
			B = 0;
		} else if (RGBPosition < 768) {
			R = 0;
			G = 255;
			B = (RGBPosition & 0xFF);
		} else if (RGBPosition < 1024) {
			R = 0;
			G = 255 - (RGBPosition & 0xFF);
			B = 255;
		} else if (RGBPosition < 1280) {
			R = (RGBPosition & 0xFF);
			G = 0;
			B = 255;
		} else if ((RGBPosition < 1536)){
			R = 255;
			G = 0;
			B = 255 - (RGBPosition & 0xFF);
		}
		Neopixel_Set(Neopixel_led,i, R, G, B);
		RGBPositionInit += (uint32_t)(1536/NUM_LED);
	}
	Neopixel_Sent(Neopixel_led);
	Neopixel_led->TimeEffect++;
}
