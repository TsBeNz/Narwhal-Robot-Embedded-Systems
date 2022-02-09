/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.c
  * @brief          : Main program body
  ******************************************************************************
  * @attention
  *
  * <h2><center>&copy; Copyright (c) 2021 STMicroelectronics.
  * All rights reserved.</center></h2>
  *
  * This software component is licensed by ST under BSD 3-Clause license,
  * the "License"; You may not use this file except in compliance with the
  * License. You may obtain a copy of the License at:
  *                        opensource.org/licenses/BSD-3-Clause
  *
  ******************************************************************************
  */
/* USER CODE END Header */
/* Includes ------------------------------------------------------------------*/
#include "main.h"
#include "adc.h"
#include "crc.h"
#include "dma.h"
#include "i2c.h"
#include "spi.h"
#include "tim.h"
#include "usart.h"
#include "gpio.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include <math.h>
#include "kinematics.h"
#include "control.h"
#include "AS5047U.h"

//#include "arm_math.h"
/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */

/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */


// Module 8-9 Variable Section
float Temperature;
float Temp_Calibration;

uint8_t Contorl_Flag;

TrajParameter Traj[4];
AS5047U Encoder[4];
KalmanParameter Kalman[4];
ControlParameter Control[4];
SteperParameter Stepper[4];


uint8_t traj_finish = 0;
float traj_t_set[4];
float t,T_Traj;


float pos,vel,dt_test,f_out,position_test;
uint16_t reg_out;
// Solfware timer
uint32_t Last_Update_Time_MS;

uint8_t UART1_rxBuffer[14] = {0};
uint8_t UART1_txBuffer[14] = {0};

/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/

/* USER CODE BEGIN PV */

/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
void PeriphCommonClock_Config(void);
/* USER CODE BEGIN PFP */

//uint8_t crc_uart(void);
//void Uart1_Sent(void);
void Joint_Traj(float *Position, float *Velocity);
/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */

/* USER CODE END 0 */

/**
  * @brief  The application entry point.
  * @retval int
  */
int main(void)
{
  /* USER CODE BEGIN 1 */

  /* USER CODE END 1 */

  /* MCU Configuration--------------------------------------------------------*/

  /* Reset of all peripherals, Initializes the Flash interface and the Systick. */
  HAL_Init();

  /* USER CODE BEGIN Init */

  /* USER CODE END Init */

  /* Configure the system clock */
  SystemClock_Config();

/* Configure the peripherals common clocks */
  PeriphCommonClock_Config();

  /* USER CODE BEGIN SysInit */

  /* USER CODE END SysInit */

  /* Initialize all configured peripherals */
  MX_GPIO_Init();
  MX_DMA_Init();
  MX_ADC3_Init();
  MX_TIM1_Init();
  MX_TIM3_Init();
  MX_TIM4_Init();
  MX_TIM5_Init();
  MX_TIM8_Init();
  MX_TIM13_Init();
  MX_TIM14_Init();
  MX_TIM15_Init();
  MX_TIM16_Init();
  MX_TIM17_Init();
  MX_USART1_UART_Init();
  MX_USART3_UART_Init();
  MX_SPI3_Init();
  MX_SPI4_Init();
  MX_TIM2_Init();
  MX_I2C2_Init();
  MX_CRC_Init();
  MX_TIM24_Init();
  MX_TIM23_Init();
  MX_TIM6_Init();
  MX_UART5_Init();
  /* USER CODE BEGIN 2 */
	HAL_GPIO_WritePin(GPIOD, GPIO_PIN_0, 1);
	HAL_GPIO_WritePin(GPIOD, GPIO_PIN_1, 1);
	HAL_GPIO_WritePin(GPIOD, GPIO_PIN_2, 1);
	HAL_GPIO_WritePin(GPIOD, GPIO_PIN_3, 1);
	HAL_GPIO_WritePin(GPIOD, GPIO_PIN_4, 1);
	HAL_GPIO_WritePin(GPIOD, GPIO_PIN_5, 1);
	HAL_GPIO_WritePin(GPIOD, GPIO_PIN_6, 1);
	HAL_GPIO_WritePin(GPIOE, GPIO_PIN_6, 1);

	Temp_Calibration = (110.0 - 30.0)	/ (*(unsigned short*) (0x1FF1E840) - *(unsigned short*) (0x1FF1E820));
	HAL_ADCEx_Calibration_Start(&hadc3, ADC_CALIB_OFFSET, ADC_SINGLE_ENDED);

//	HAL_UART_Receive_IT(&huart1, UART1_rxBuffer, 14);
  	while (HAL_UART_Receive_DMA(&huart1, UART1_rxBuffer, 14) != HAL_OK)

	AS5047U_init(&Encoder[0], &hspi3, GPIOD, &hcrc, GPIO_PIN_5);
	Kalman_init(&Kalman[0], 5000,0.001);
	CascadeControl_init(&Control[0], 0.5, 0, 0, 0.2, 0, 0, 9, 1600);
	Step_Driver_init(&Stepper[0], &htim15, TIM_CHANNEL_1, GPIOE, GPIO_PIN_3, 500000, 0);
	HAL_TIM_Base_Start_IT(&htim23);
	/*Go2Home*/
//	while (1) {
//		if (Contorl_Flag) {
//			float Position;
//			AS5047U_Position_Highspeed_Read(&Encoder[0]);
//			position_test = (float) (Encoder[0].Position) * PI / 8192.0f;
//			CascadeControl(&Control[0], &Kalman[0], position_test, Position,0);
//			Step_Driver(&Stepper[0], Control[0].Output);
//			Contorl_Flag = 0;
//		}
//		if (Con){
//
//		}
//	}


  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  while (1)
  {
    /* USER CODE END WHILE */

    /* USER CODE BEGIN 3 */
		if (Contorl_Flag) {
			float Position, Velocity;
			Joint_Traj(&Position,&Velocity);
			AS5047U_Position_Highspeed_Read(&Encoder[0]);
			position_test = (float) (Encoder[0].Position) * PI / 8192.0f;
			CascadeControl(&Control[0], &Kalman[0], position_test,Position,Velocity);
			Step_Driver(&Stepper[0], Control[0].Output);
			Contorl_Flag = 0;
		}
		if (traj_finish){
			if(Control[0].PositionFeedback > 1.5){
				Traj_Coeff_Cal(&Traj[0], 5, 0.5, Control[0].PositionFeedback, Control[0].VelocityFeedback);
			}
			else{
				Traj_Coeff_Cal(&Traj[0], 5, 1.6, Control[0].PositionFeedback, Control[0].VelocityFeedback);
			}
			traj_finish = 0;
			T_Traj = 5;
		}
		int a = HAL_GetTick();
		if (a - Last_Update_Time_MS >= 1000) {
			Last_Update_Time_MS = a;
			HAL_ADC_Start_IT(&hadc3); 					//read temperature sensor
			HAL_GPIO_TogglePin(GPIOC, GPIO_PIN_9);
		}
	}
  /* USER CODE END 3 */
}

/**
  * @brief System Clock Configuration
  * @retval None
  */
void SystemClock_Config(void)
{
  RCC_OscInitTypeDef RCC_OscInitStruct = {0};
  RCC_ClkInitTypeDef RCC_ClkInitStruct = {0};

  /** Supply configuration update enable
  */
  HAL_PWREx_ConfigSupply(PWR_LDO_SUPPLY);
  /** Configure the main internal regulator output voltage
  */
  __HAL_PWR_VOLTAGESCALING_CONFIG(PWR_REGULATOR_VOLTAGE_SCALE0);

  while(!__HAL_PWR_GET_FLAG(PWR_FLAG_VOSRDY)) {}
  /** Macro to configure the PLL clock source
  */
  __HAL_RCC_PLL_PLLSOURCE_CONFIG(RCC_PLLSOURCE_HSE);
  /** Initializes the RCC Oscillators according to the specified parameters
  * in the RCC_OscInitTypeDef structure.
  */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSE;
  RCC_OscInitStruct.HSEState = RCC_HSE_ON;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSE;
  RCC_OscInitStruct.PLL.PLLM = 2;
  RCC_OscInitStruct.PLL.PLLN = 44;
  RCC_OscInitStruct.PLL.PLLP = 1;
  RCC_OscInitStruct.PLL.PLLQ = 2;
  RCC_OscInitStruct.PLL.PLLR = 2;
  RCC_OscInitStruct.PLL.PLLRGE = RCC_PLL1VCIRANGE_3;
  RCC_OscInitStruct.PLL.PLLVCOSEL = RCC_PLL1VCOWIDE;
  RCC_OscInitStruct.PLL.PLLFRACN = 0;
  if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK)
  {
    Error_Handler();
  }
  /** Initializes the CPU, AHB and APB buses clocks
  */
  RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK|RCC_CLOCKTYPE_SYSCLK
                              |RCC_CLOCKTYPE_PCLK1|RCC_CLOCKTYPE_PCLK2
                              |RCC_CLOCKTYPE_D3PCLK1|RCC_CLOCKTYPE_D1PCLK1;
  RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_PLLCLK;
  RCC_ClkInitStruct.SYSCLKDivider = RCC_SYSCLK_DIV1;
  RCC_ClkInitStruct.AHBCLKDivider = RCC_HCLK_DIV2;
  RCC_ClkInitStruct.APB3CLKDivider = RCC_APB3_DIV2;
  RCC_ClkInitStruct.APB1CLKDivider = RCC_APB1_DIV2;
  RCC_ClkInitStruct.APB2CLKDivider = RCC_APB2_DIV2;
  RCC_ClkInitStruct.APB4CLKDivider = RCC_APB4_DIV2;

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_3) != HAL_OK)
  {
    Error_Handler();
  }
}

/**
  * @brief Peripherals Common Clock Configuration
  * @retval None
  */
void PeriphCommonClock_Config(void)
{
  RCC_PeriphCLKInitTypeDef PeriphClkInitStruct = {0};

  /** Initializes the peripherals clock
  */
  PeriphClkInitStruct.PeriphClockSelection = RCC_PERIPHCLK_ADC|RCC_PERIPHCLK_SPI4
                              |RCC_PERIPHCLK_UART5|RCC_PERIPHCLK_USART1
                              |RCC_PERIPHCLK_USART3;
  PeriphClkInitStruct.PLL2.PLL2M = 2;
  PeriphClkInitStruct.PLL2.PLL2N = 16;
  PeriphClkInitStruct.PLL2.PLL2P = 2;
  PeriphClkInitStruct.PLL2.PLL2Q = 2;
  PeriphClkInitStruct.PLL2.PLL2R = 2;
  PeriphClkInitStruct.PLL2.PLL2RGE = RCC_PLL2VCIRANGE_3;
  PeriphClkInitStruct.PLL2.PLL2VCOSEL = RCC_PLL2VCOWIDE;
  PeriphClkInitStruct.PLL2.PLL2FRACN = 0;
  PeriphClkInitStruct.Spi45ClockSelection = RCC_SPI45CLKSOURCE_PLL2;
  PeriphClkInitStruct.Usart234578ClockSelection = RCC_USART234578CLKSOURCE_PLL2;
  PeriphClkInitStruct.Usart16ClockSelection = RCC_USART16910CLKSOURCE_PLL2;
  PeriphClkInitStruct.AdcClockSelection = RCC_ADCCLKSOURCE_PLL2;
  if (HAL_RCCEx_PeriphCLKConfig(&PeriphClkInitStruct) != HAL_OK)
  {
    Error_Handler();
  }
}

/* USER CODE BEGIN 4 */

//void HAL_UART_RxCpltCallback(UART_HandleTypeDef *huart)
//{
//	Uart_Flag |= 0x01;
//    HAL_UART_Receive_IT(&huart1, UART1_rxBuffer, 14);
//}


//void HAL_UART_ErrorCallback(UART_HandleTypeDef *huart){
////	__HAL_UART_CLEAR_OREFLAG(huart);
////    HAL_UART_DeInit();
//    HAL_UART_Receive_DMA(&huart1, UART1_rxBuffer, 14);
//}

void HAL_ADC_ConvCpltCallback(ADC_HandleTypeDef *hadc) {
	if (hadc == &hadc3) {
		uint32_t ADC_Output = HAL_ADC_GetValue(&hadc3);
		Temperature = Temp_Calibration
				* (float) ((float) ADC_Output - *(unsigned short*) (0x1FF1E820))
				+ 30.0f;
	}
}

void HAL_TIM_PeriodElapsedCallback(TIM_HandleTypeDef *htim) {
	if (htim == &htim23) {
		Contorl_Flag = 1;
	}
}

void Joint_Traj(float *Position, float *Velocity){
	if(!traj_finish){
		float traj_t_set[5];
		traj_t_set[0] = t;
		traj_t_set[1] = t*t;
		traj_t_set[2] = traj_t_set[1]*t;
		traj_t_set[3] = traj_t_set[2]*t;
		traj_t_set[4] = traj_t_set[3]*t;
		/*for loop For 4 Join*/
		TrajFollow(&Traj[0], traj_t_set, Position, Velocity);
		/*for loop For 4 Join*/
		t += delta_t;
		if (t > T_Traj){
			traj_finish = 1;
			t = 0;
		}
	}
}

/* USER CODE END 4 */

/**
  * @brief  This function is executed in case of error occurrence.
  * @retval None
  */
void Error_Handler(void)
{
  /* USER CODE BEGIN Error_Handler_Debug */
  /* User can add his own implementation to report the HAL error return state */
  __disable_irq();
  while (1)
  {
  }
  /* USER CODE END Error_Handler_Debug */
}

#ifdef  USE_FULL_ASSERT
/**
  * @brief  Reports the name of the source file and the source line number
  *         where the assert_param error has occurred.
  * @param  file: pointer to the source file name
  * @param  line: assert_param error line source number
  * @retval None
  */
void assert_failed(uint8_t *file, uint32_t line)
{
  /* USER CODE BEGIN 6 */
  /* User can add his own implementation to report the file name and line number,
     ex: printf("Wrong parameters value: file %s on line %d\r\n", file, line) */
  /* USER CODE END 6 */
}
#endif /* USE_FULL_ASSERT */

