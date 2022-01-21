/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.h
  * @brief          : Header for main.c file.
  *                   This file contains the common defines of the application.
  ******************************************************************************
  * @attention
  *
  * Copyright (c) 2022 STMicroelectronics.
  * All rights reserved.
  *
  * This software is licensed under terms that can be found in the LICENSE file
  * in the root directory of this software component.
  * If no LICENSE file comes with this software, it is provided AS-IS.
  *
  ******************************************************************************
  */
/* USER CODE END Header */

/* Define to prevent recursive inclusion -------------------------------------*/
#ifndef __MAIN_H
#define __MAIN_H

#ifdef __cplusplus
extern "C" {
#endif

/* Includes ------------------------------------------------------------------*/
#include "stm32h7xx_hal.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */

/* USER CODE END Includes */

/* Exported types ------------------------------------------------------------*/
/* USER CODE BEGIN ET */

/* USER CODE END ET */

/* Exported constants --------------------------------------------------------*/
/* USER CODE BEGIN EC */

/* USER CODE END EC */

/* Exported macro ------------------------------------------------------------*/
/* USER CODE BEGIN EM */

/* USER CODE END EM */

void HAL_TIM_MspPostInit(TIM_HandleTypeDef *htim);

/* Exported functions prototypes ---------------------------------------------*/
void Error_Handler(void);

/* USER CODE BEGIN EFP */

/* USER CODE END EFP */

/* Private defines -----------------------------------------------------------*/
#define DIR3_Pin GPIO_PIN_2
#define DIR3_GPIO_Port GPIOE
#define DIR4_Pin GPIO_PIN_3
#define DIR4_GPIO_Port GPIOE
#define DIR5_Pin GPIO_PIN_4
#define DIR5_GPIO_Port GPIOE
#define STEP3_Pin GPIO_PIN_5
#define STEP3_GPIO_Port GPIOE
#define Output_EN_Pin GPIO_PIN_6
#define Output_EN_GPIO_Port GPIOE
#define ADC4_Pin GPIO_PIN_0
#define ADC4_GPIO_Port GPIOC
#define ADC3_Pin GPIO_PIN_1
#define ADC3_GPIO_Port GPIOC
#define ADC2_Pin GPIO_PIN_2
#define ADC2_GPIO_Port GPIOC
#define ADC1_Pin GPIO_PIN_3
#define ADC1_GPIO_Port GPIOC
#define ENC1A_Pin GPIO_PIN_0
#define ENC1A_GPIO_Port GPIOA
#define ENC1B_Pin GPIO_PIN_1
#define ENC1B_GPIO_Port GPIOA
#define STEP1_Pin GPIO_PIN_6
#define STEP1_GPIO_Port GPIOA
#define STEP2_Pin GPIO_PIN_7
#define STEP2_GPIO_Port GPIOA
#define Griper0_Pin GPIO_PIN_4
#define Griper0_GPIO_Port GPIOC
#define Griper1_Pin GPIO_PIN_5
#define Griper1_GPIO_Port GPIOC
#define Griper2_Pin GPIO_PIN_0
#define Griper2_GPIO_Port GPIOB
#define EMSW0_Pin GPIO_PIN_1
#define EMSW0_GPIO_Port GPIOB
#define EMSW1_Pin GPIO_PIN_2
#define EMSW1_GPIO_Port GPIOB
#define EMSW2_Pin GPIO_PIN_7
#define EMSW2_GPIO_Port GPIOE
#define EMSW3_Pin GPIO_PIN_8
#define EMSW3_GPIO_Port GPIOE
#define EMSW4_Pin GPIO_PIN_9
#define EMSW4_GPIO_Port GPIOE
#define EERAM_SS_Pin GPIO_PIN_11
#define EERAM_SS_GPIO_Port GPIOE
#define SPI4_SCK_Pin GPIO_PIN_12
#define SPI4_SCK_GPIO_Port GPIOE
#define EERAM_HOLD_Pin GPIO_PIN_15
#define EERAM_HOLD_GPIO_Port GPIOE
#define ENC2A_Pin GPIO_PIN_12
#define ENC2A_GPIO_Port GPIOD
#define ENC2B_Pin GPIO_PIN_13
#define ENC2B_GPIO_Port GPIOD
#define LED4_Pin GPIO_PIN_14
#define LED4_GPIO_Port GPIOD
#define LED3_Pin GPIO_PIN_15
#define LED3_GPIO_Port GPIOD
#define ENC3A_Pin GPIO_PIN_6
#define ENC3A_GPIO_Port GPIOC
#define ENC3B_Pin GPIO_PIN_7
#define ENC3B_GPIO_Port GPIOC
#define LED2_Pin GPIO_PIN_8
#define LED2_GPIO_Port GPIOC
#define LED1_Pin GPIO_PIN_9
#define LED1_GPIO_Port GPIOC
#define ENC4A_Pin GPIO_PIN_8
#define ENC4A_GPIO_Port GPIOA
#define ENC4B_Pin GPIO_PIN_9
#define ENC4B_GPIO_Port GPIOA
#define ENC6I_Pin GPIO_PIN_10
#define ENC6I_GPIO_Port GPIOA
#define ENC6A_Pin GPIO_PIN_15
#define ENC6A_GPIO_Port GPIOA
#define SPI3_SS6_Pin GPIO_PIN_0
#define SPI3_SS6_GPIO_Port GPIOD
#define SPI3_SS5_Pin GPIO_PIN_1
#define SPI3_SS5_GPIO_Port GPIOD
#define SPI3_SS4_Pin GPIO_PIN_2
#define SPI3_SS4_GPIO_Port GPIOD
#define SPI3_SS3_Pin GPIO_PIN_3
#define SPI3_SS3_GPIO_Port GPIOD
#define SPI3_SS2_Pin GPIO_PIN_4
#define SPI3_SS2_GPIO_Port GPIOD
#define SPI3_SS1_Pin GPIO_PIN_5
#define SPI3_SS1_GPIO_Port GPIOD
#define SPI3_EN_Pin GPIO_PIN_6
#define SPI3_EN_GPIO_Port GPIOD
#define ENC6B_Pin GPIO_PIN_3
#define ENC6B_GPIO_Port GPIOB
#define ENC5A_Pin GPIO_PIN_4
#define ENC5A_GPIO_Port GPIOB
#define ENC5B_Pin GPIO_PIN_5
#define ENC5B_GPIO_Port GPIOB
#define EN45_Pin GPIO_PIN_6
#define EN45_GPIO_Port GPIOB
#define EN123_Pin GPIO_PIN_7
#define EN123_GPIO_Port GPIOB
#define STEP4_Pin GPIO_PIN_8
#define STEP4_GPIO_Port GPIOB
#define STEP5_Pin GPIO_PIN_9
#define STEP5_GPIO_Port GPIOB
#define DIR1_Pin GPIO_PIN_0
#define DIR1_GPIO_Port GPIOE
#define DIR2_Pin GPIO_PIN_1
#define DIR2_GPIO_Port GPIOE
/* USER CODE BEGIN Private defines */

/* USER CODE END Private defines */

#ifdef __cplusplus
}
#endif

#endif /* __MAIN_H */
