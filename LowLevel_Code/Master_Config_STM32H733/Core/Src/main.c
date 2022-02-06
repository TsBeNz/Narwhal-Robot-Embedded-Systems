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

AS5047U Encoder;



// Solfware timer
uint32_t Last_Update_Time_MS;

struct System_Status_Type {
	float Temperature;
	float Vin;
};
struct System_Status_Type System_Status;

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

//void CascadeControl(void);
//uint16_t Encoder_Position_SPI(GPIO_TypeDef *GPIOx, uint16_t GPIO_Pin);
//int16_t Encoder_Speed_SPI(GPIO_TypeDef *GPIOx, uint16_t GPIO_Pin);
//void Encoder_Setup();
//void Encoder_command(GPIO_TypeDef *GPIOx, uint16_t GPIO_Pin,uint16_t Address, uint16_t command_input);
//void StartUp_Init_Parameter(void);
//uint8_t crc_uart(void);
//void Uart1_Sent(void);

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
	HAL_TIM_Base_Start(&htim24); 		// Ms delay timer

	HAL_Delay(10);
//	Contorl_Flag |= 0x02; //Use QEI

//	Encoder_Setup(); // Change Resolution ABI to 14 bits

	HAL_ADCEx_Calibration_Start(&hadc3, ADC_CALIB_OFFSET, ADC_SINGLE_ENDED);

//	HAL_UART_Receive_IT(&huart1, UART1_rxBuffer, 14);
  	while (HAL_UART_Receive_DMA(&huart1, UART1_rxBuffer, 14) != HAL_OK)

	HAL_TIM_PWM_Start(&htim13, TIM_CHANNEL_1);
	HAL_TIM_PWM_Start(&htim14, TIM_CHANNEL_1);
	HAL_TIM_PWM_Start(&htim15, TIM_CHANNEL_1);
	HAL_TIM_PWM_Start(&htim15, TIM_CHANNEL_2);
	HAL_TIM_PWM_Start(&htim16, TIM_CHANNEL_1);
	HAL_TIM_PWM_Start(&htim17, TIM_CHANNEL_1);

	HAL_TIM_Encoder_Start(&htim1, TIM_CHANNEL_ALL);
	HAL_TIM_Encoder_Start(&htim2, TIM_CHANNEL_ALL);
	HAL_TIM_Encoder_Start(&htim3, TIM_CHANNEL_ALL);
	HAL_TIM_Encoder_Start(&htim4, TIM_CHANNEL_ALL);
	HAL_TIM_Encoder_Start(&htim5, TIM_CHANNEL_ALL);
	HAL_TIM_Encoder_Start(&htim8, TIM_CHANNEL_ALL);

	HAL_TIM_Base_Start(&htim6);			// ตัวจับเวลา
	HAL_TIM_Base_Start_IT(&htim23); 	// Interrupt Timer
	AS5047U_init(&Encoder, &hspi3, GPIOD, &hcrc, GPIO_PIN_5);

  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  while (1)
  {
    /* USER CODE END WHILE */

    /* USER CODE BEGIN 3 */
//		if (Uart_Flag & 0x01) {
//			Uart_Flag &= 0xFE; // Clear Flag
//			if (UART1_rxBuffer[0] & 0xFF && UART1_rxBuffer[1] & 0xFF) {
//				uint8_t Crc_Uart_Output = crc_uart();
//				if (UART1_rxBuffer[13] == Crc_Uart_Output) {
//					uint8_t Ping_Command_Buffer[14] = { 255, 255, 0x11, 255, 255, 0, 0, 0, 0, 0, 0, 0, 0, 0 };
//					switch (UART1_rxBuffer[2]) {
//					case 0x11:
//						// ping
//						Uart1_Sent();
//						break;
//					default:
//						// Command Error
//						break;
//					}
//				} else {
//					// Checksum error
//					uint8_t Error_Checksum_Buffer[14] = { 255, 255, 70, 97, 105,	108, 33, 33, 0, 0, 0, 0, 0, 0 };
//
//					HAL_UART_Transmit(&huart1, UART1_txBuffer, 14, 100);
//				}
//			} else{
//				// Header error เเ�?้  Command ด้วย
//				uint8_t Error_Header_Buffer[14] = { 255, 255, 70, 97, 105,	108, 33, 33, 0, 0, 0, 0, 0, 0 };
//
//				HAL_UART_Transmit(&huart1, UART1_txBuffer, 14, 100);
//			}
//		}
////		float32_t c = 0.1;
////		float32_t d = arm_cos_f32(c);
//		test_encoder_QEI[0] = TIM5->CNT;
//		test_encoder_QEI[1] = TIM4->CNT;
//		test_encoder_QEI[2] = TIM8->CNT;
//		test_encoder_QEI[3] = TIM1->CNT;
//		test_encoder_QEI[4] = TIM3->CNT;
//		test_encoder_QEI[5] = TIM2->CNT;
//		output_spi_test2 = Encoder_Position_SPI(GPIOD,GPIO_PIN_5);
//
		int a = HAL_GetTick();
		if (a - Last_Update_Time_MS >= 100) {
			Last_Update_Time_MS = a;
			AS5047U_Position_Highspeed_Read(&Encoder);

//			HAL_ADC_Start_IT(&hadc3); //read temperature sensor
//			HAL_GPIO_TogglePin(GPIOC, GPIO_PIN_8);
//			TIM13->ARR += 30;
//			TIM13->CCR1 = TIM13->ARR/2;
//			TIM14->ARR += 30;
//			TIM14->CCR1 = TIM14->ARR/2;
//			TIM15->ARR += 30;
//			TIM15->CCR2 = TIM15->ARR/2;
//			TIM16->ARR += 30;
//			TIM16->CCR1 = TIM16->ARR/2;
//			TIM17->ARR += 30;
//			TIM17->CCR1 = TIM17->ARR/2;

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
//
//void HAL_UART_ErrorCallback(UART_HandleTypeDef *huart){
////	__HAL_UART_CLEAR_OREFLAG(huart);
////    HAL_UART_DeInit();
//    HAL_UART_Receive_DMA(&huart1, UART1_rxBuffer, 14);
//}
//
//void HAL_ADC_ConvCpltCallback(ADC_HandleTypeDef *hadc){
//	uint32_t ADC_Output = HAL_ADC_GetValue(&hadc3);
//	System_Status.Temperature = Temp_Calibration * (float)((float)ADC_Output - *(unsigned short*) (0x1FF1E820)) + 30.0f;
////	System_Status.Vin = 0.0008056640625f*ADC_Output;
//}
//
//void HAL_TIM_PeriodElapsedCallback(TIM_HandleTypeDef *htim) {
//	if (htim == &htim23) {
//		//control loop
//		count_time_blink_led++;
//		if(count_time_blink_led>=100){
//			count_time_blink_led = 0;
//		}
//		Contorl_Flag |= 0x01;
//	}
//}

//void Uart1_Sent(){
//	UART1_txBuffer[13] = crc_uart();
//	HAL_UART_Transmit(&huart1, UART1_txBuffer, 14, 100);
//}
//
//inline uint8_t crc_uart(){
//	return (uint8_t) HAL_CRC_Calculate(&hcrc, (uint32_t*)UART1_rxBuffer, 13) ^ 0xFF;
//}
//
//inline void CascadeControl() {
//
//	static float TrajectoryTime = 0;
//	static float TrajectoryTimeSetpoint = 0;
//
//	// Update Encoder Position
//
//	int JointCount = 0;
//	float TrajectoryTimePow2 = TrajectoryTime * TrajectoryTime;
//	float TrajectoryTimePow3 = TrajectoryTimePow2 * TrajectoryTime;
//	float TrajectoryTimePow4 = TrajectoryTimePow3 * TrajectoryTime;
//	float TrajectoryTimePow5 = TrajectoryTimePow4 * TrajectoryTime;
//
//	// Trajectory Motion Calculation
//
//	if (TrajectoryMoveStatus) {
//		for (JointCount = 0; JointCount < 5; JointCount++) {
//		Joint[JointCount].PositionPIDInput = Joint[JointCount].TrajectoryCoefficient[0]	+ (Joint[JointCount].TrajectoryCoefficient[1] * TrajectoryTime) + (Joint[JointCount].TrajectoryCoefficient[3] * TrajectoryTimePow3) + (Joint[JointCount].TrajectoryCoefficient[4] * TrajectoryTimePow4) + (Joint[JointCount].TrajectoryCoefficient[5] * TrajectoryTimePow5);
//		Joint[JointCount].VelocitySetpoint = Joint[JointCount].TrajectoryCoefficient[1] + (3 * Joint[JointCount].TrajectoryCoefficient[3] * TrajectoryTimePow2) + (4 * Joint[JointCount].TrajectoryCoefficient[4] * TrajectoryTimePow3) + (5 * Joint[JointCount].TrajectoryCoefficient[5] * TrajectoryTimePow4);
//		}
//	}
//
//
//	// Only Joint 1-4 (Stepper with Encoder Feedback)
//	for (JointCount = 0; JointCount < 4 ; JointCount++)
//	{
//		//Velocity Fix Window
//		float DeltaPosition = Joint[JointCount].EncoderPosition[0] - Joint[JointCount].EncoderPosition[1];
//		DeltaPosition /= 1000;
//		Joint[JointCount].VelocityFixWindow = DeltaPosition / ControlLoopTime;
//
//		//Velocity Estimation (Input By Velocity)
//		float Q = sigma_a[JointCount] * sigma_a[JointCount];
//		float R = sigma_w[JointCount] * sigma_w[JointCount];
//		Joint[JointCount].EstimatePosition[0] = Joint[JointCount].EstimatePosition[0] + Joint[JointCount].EstimateVelocity[1] * ControlLoopTime;
//		Joint[JointCount].EstimateVelocity[0] = 0 + Joint[JointCount].EstimateVelocity[1];
//		float ye = Joint[JointCount].VelocityFixWindow - Joint[JointCount].EstimateVelocity[0];  // Input
//		p11[JointCount] = p11[JointCount] + ControlLoopTime * p21[JointCount] + (Q * ControlLoopTimePow4) / 4 + (ControlLoopTimePow2 * (p12[JointCount] + ControlLoopTime * p22[JointCount])) / ControlLoopTime;
//		p12[JointCount] = p12[JointCount] + ControlLoopTime * p22[JointCount] + (Q * ControlLoopTimePow3) / 2;
//		p21[JointCount] = (2 * ControlLoopTime * p21[JointCount] + Q * ControlLoopTimePow2 + 2 * p22[JointCount] * ControlLoopTimePow2) / (2 * ControlLoopTime);
//		p22[JointCount] = Q * ControlLoopTimePow2 + p22[JointCount];
//		Joint[JointCount].EstimatePosition[0] += (p12[JointCount] * ye) / (R + p22[JointCount]);
//		Joint[JointCount].EstimateVelocity[0] = Joint[JointCount].EstimateVelocity[0] + (p22[JointCount] * ye) / (R + p22[JointCount]);
//		p11[JointCount] = p11[JointCount] - (p12[JointCount] * p21[JointCount]) / (R + p22[JointCount]);
//		p12[JointCount] = p12[JointCount] - (p12[JointCount] * p22[JointCount]) / (R + p22[JointCount]);
//		p21[JointCount] = -p21[JointCount] * (p22[JointCount] / (R + p22[JointCount]) - 1);
//		p22[JointCount] = -p22[JointCount] * (p22[JointCount] / (R + p22[JointCount]) - 1);
//
//		// Position Controller
//		Joint[JointCount].PositionError[0] = Joint[JointCount].PositionPIDInput - Joint[JointCount].EncoderPosition[0];
//		Joint[JointCount].PositionPIDOutput = Joint[JointCount].PositionError[0] * Joint[JointCount].PositionPConstant;
//
//		// Velocity Controller
//		if (Joint[JointCount].ControlEnable[1] == 1) {
//			Joint[JointCount].VelocitySetpoint += Joint[JointCount].PositionPIDOutput;
//		}
//		Joint[JointCount].VelocityError[0] = Joint[JointCount].VelocitySetpoint - Joint[JointCount].EstimateVelocity[0];
//		Joint[JointCount].VelocityITerm += Joint[JointCount].VelocityError[0];
//		Joint[JointCount].VelocityPIDOutput = (Joint[JointCount].VelocityError[0] * Joint[JointCount].VelocityPIDConstant[0]) + (Joint[JointCount].VelocityITerm * Joint[JointCount].VelocityPIDConstant[1]) + ((Joint[JointCount].VelocityError[0] - Joint[JointCount].VelocityError[1]) * Joint[JointCount].VelocityPIDConstant[2]);
//
//		//Update Parameter
//		Joint[JointCount].EncoderPosition[1] = Joint[JointCount].EncoderPosition[0];
//		Joint[JointCount].PositionError[1] = Joint[JointCount].PositionError[0];
//		Joint[JointCount].VelocityError[1] = Joint[JointCount].VelocityError[0];
//		Joint[JointCount].EstimatePosition[1] = Joint[JointCount].EstimatePosition[0];
//		Joint[JointCount].EstimateVelocity[1] = Joint[JointCount].EstimateVelocity[0];
//
//
//		if(Joint[JointCount].VelocityPIDOutput > 10000){
//			Joint[JointCount].VelocityPIDOutput = 10000;
//		}
//		else if(Joint[JointCount].VelocityPIDOutput < -10000){
//			Joint[JointCount].VelocityPIDOutput = 10000;
//		}
//
//	}
//
//	// Update Trajectory Path & Parameter
//	if (TrajectoryNewSetpoint) {
//
//		// time set point calculation
//		TrajectoryTimeSetpoint = 10;
//		// time set point?
//
//		float TrajectoryTimeSetpointPow2 = TrajectoryTimeSetpoint * TrajectoryTimeSetpoint;
//		float TrajectoryTimeSetpointPow3 = TrajectoryTimeSetpointPow2 * TrajectoryTimeSetpoint;
//		float TrajectoryTimeSetpointPow4 = TrajectoryTimeSetpointPow3 * TrajectoryTimeSetpoint;
//		float TrajectoryTimeSetpointPow5 = TrajectoryTimeSetpointPow4 * TrajectoryTimeSetpoint;
//		for (JointCount = 0; JointCount < 5; JointCount++) {
//			float ds = Joint[JointCount].EncoderPosition[0] - Joint[JointCount].PositionSetpoint;
//			float tfv0 = TrajectoryTimeSetpoint * Joint[JointCount].EstimateVelocity[0];
//
//			Joint[JointCount].TrajectoryCoefficient[0] = Joint[JointCount].EncoderPosition[0];
//			Joint[JointCount].TrajectoryCoefficient[1] = Joint[JointCount].EstimateVelocity[0];
//			Joint[JointCount].TrajectoryCoefficient[3] = -(2*(5*ds + 3*tfv0))/TrajectoryTimeSetpointPow3;
//			Joint[JointCount].TrajectoryCoefficient[4] = (15 * ds + 8 * tfv0)/TrajectoryTimeSetpointPow4;
//			Joint[JointCount].TrajectoryCoefficient[5] = -(3*(2* ds + tfv0))/TrajectoryTimeSetpointPow5;
//
//		}
//		TrajectoryMoveStatus = 1;
//		TrajectoryNewSetpoint = 0;
//		TrajectoryTime = 0;
//	}
//	else {
//		if (TrajectoryMoveStatus) {
//			TrajectoryTime += ControlLoopTime;
//			if (TrajectoryTime > TrajectoryTimeSetpoint) {
//				TrajectoryTime = 0;
//				TrajectoryMoveStatus = 0;
//			}
//		}
//	}
//}
//
//
//
//void StartUp_Init_Parameter(void){
//	// Control Parameter
//	Joint[0].PositionPConstant = 1;
//	Joint[0].VelocityPIDConstant[0] = 1;
//	Joint[0].VelocityPIDConstant[1] = 0;
//	Joint[0].VelocityPIDConstant[2] = 0;
//	Joint[0].DigitalIO_Port[0] = GPIOD;		// SPI CS
//	Joint[0].DigitalIO_Port[1] = GPIOE;		// DIR
//	Joint[0].DigitalIO_Pin[0] = GPIO_PIN_5;
//	Joint[0].DigitalIO_Pin[1] = GPIO_PIN_0;
//	Joint[0].TIMx_PWM = TIM13;
//	Joint[0].TIMx_ENC = TIM5;
//
//	Joint[1].PositionPConstant = 1;
//	Joint[1].VelocityPIDConstant[0] = 1;
//	Joint[1].VelocityPIDConstant[1] = 0;
//	Joint[1].VelocityPIDConstant[2] = 0;
//	Joint[1].DigitalIO_Port[0] = GPIOD;		// SPI CS
//	Joint[1].DigitalIO_Port[1] = GPIOE;		// DIR
//	Joint[1].DigitalIO_Pin[0] = GPIO_PIN_4;
//	Joint[1].DigitalIO_Pin[1] = GPIO_PIN_1;
//	Joint[1].TIMx_PWM = TIM14;
//	Joint[0].TIMx_ENC = TIM4;
//
//	Joint[2].PositionPConstant = 1;
//	Joint[2].VelocityPIDConstant[0] = 1;
//	Joint[2].VelocityPIDConstant[1] = 0;
//	Joint[2].VelocityPIDConstant[2] = 0;
//	Joint[2].DigitalIO_Port[0] = GPIOD;		// SPI CS
//	Joint[2].DigitalIO_Port[1] = GPIOE;		// DIR
//	Joint[2].DigitalIO_Pin[0] = GPIO_PIN_3;
//	Joint[2].DigitalIO_Pin[1] = GPIO_PIN_3;
//	Joint[2].TIMx_PWM = TIM16;
//	Joint[0].TIMx_ENC = TIM8;
//
//	Joint[3].PositionPConstant = 1;
//	Joint[3].VelocityPIDConstant[0] = 1;
//	Joint[3].VelocityPIDConstant[1] = 0;
//	Joint[3].VelocityPIDConstant[2] = 0;
//	Joint[3].DigitalIO_Port[0] = GPIOD;		// SPI CS
//	Joint[3].DigitalIO_Port[1] = GPIOE;		// DIR
//	Joint[3].DigitalIO_Pin[0] = GPIO_PIN_2;
//	Joint[3].DigitalIO_Pin[1] = GPIO_PIN_4;
//	Joint[3].TIMx_PWM = TIM17;
//	Joint[0].TIMx_ENC = TIM1;
//
//	Joint[4].PositionPConstant = 1;
//	Joint[4].VelocityPIDConstant[0] = 1;
//	Joint[4].VelocityPIDConstant[1] = 0;
//	Joint[4].VelocityPIDConstant[2] = 0;
//	Joint[4].DigitalIO_Port[0] = GPIOD;		// SPI CS
//	Joint[4].DigitalIO_Port[1] = GPIOE;		// DIR
//	Joint[4].DigitalIO_Pin[0] = GPIO_PIN_1;
//	Joint[4].DigitalIO_Pin[1] = GPIO_PIN_2;
//	Joint[4].TIMx_PWM = TIM13;
//	Joint[0].TIMx_ENC = TIM3;
//
//	for (int k = 0; k < 5; k++) {
//		Joint[k].ControlEnable[0] = 1;
//		Joint[k].ControlEnable[1] = 1;
//	}
//
//	Temp_Calibration =
//			(110.0 - 30.0)
//					/ (*(unsigned short*) (0x1FF1E840)
//							- *(unsigned short*) (0x1FF1E820));
//}


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

