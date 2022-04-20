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
#include "Neopixel.h"
#include "TPM75.h"

//#include "arm_math.h"
/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */

/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */

/*   Module 8-9 Variable Section   */


float Z_TopOffset = 300;
float Z_2GripOffset = 150;


float Temperature;
float Temp_Calibration;

typedef enum
{
	Chess_idle,
	Move_2_Start_Top_Point,
	Move_2_Start_Grip_Point,
	Move_2_Start_Point_and_Griping,
	Back_2_Start_Top_Point,
	Move_2_End_Top_Point,
	Move_2_End_Ungrip_Point,
	Move_2_End_Point_and_Ungriping,
	Back_2_End_Top_Point,
	ChessMove_Finish,
} ChessMoveState;

/*  Library */
AS5047U Encoder[4];
KalmanParameter Kalman[5];
ControlParameter Control[4];
SteperParameter Stepper[4];
NeopixelParameter Neopixel;
TPM75 TempSensor;
TrajParameter Traj[7]; /* index 0-3 is joint Traj and 4 is Z axis Traj */

float SetPoint_Position[5];
float SetPoint_Velocity[5];
float t;

/*   Flag   */
uint8_t Contorl_Flag;
uint8_t Traj_Flag;
uint8_t Protocol_Flag;
uint8_t Chessmove_State;
uint8_t Trajz_Flag;

/*	Software Timer	*/
uint32_t Software_Timer_1s;
uint32_t Software_Timer_100ms;

uint8_t UART5_rxBuffer[14] __attribute__((section("RAM_D2"))) = {0};
uint8_t UART5_txBuffer[36] __attribute__((section("RAM_D2"))) = {0};
uint8_t ChessIndex[2];
uint8_t Chess_Move_Start_Flag = 0;
ChessMoveState ChessMoveStates = Chess_idle;

float tune_PID[2] = {0,0};
float T_tune_PID = 2;


/*	Jog Variable	*/
float JointTrajSet[4];
float TaskTrajSet[3];

uint8_t traj_finish = 0;
float traj_t_set[4];
float t,T_Traj;

float pos,vel,dt_test,f_out,position_test;
uint16_t reg_out;
float v2freqGain;

uint8_t Buffer_TPM75[2];

// Debug Variable //



uint8_t Test_traj = 0;
uint8_t Test_traj2 = 0;
float Test_traj_Val[4];
float Time_Live_Ex1 = 2;
float TaskSpace_Live_Ex1[3] ={0,0,0};


/*	ChessMove	*/
float EndEffectorTarget[3];

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

uint8_t CRC8(uint8_t *Data,uint8_t BufferLength);
void SentData(uint8_t range);
void Narwhal_Protocol();
void Control_Function();
void JMoveTaskSpace(float Task2Go[3], float Time2Move);
void LMoveTaskSpace(float Task2Go[3], float Time2Move);
void ChessNotMovePathWay(uint8_t Index2Move, float Z_Offset, uint8_t IsJMove);
void ChessMoveStateMachine();


//void Joint_Traj(float *Position, float *Velocity);
//void JointSpaceTraj(float Task2Move[3],float Time2Move);
//void Chess_Tracker(uint8_t Chess_Index, float Chess_Theta, float Chess_Omega);
//void ChessMove1(uint8_t StartIndex, uint8_t EndIndex);
//uint8_t ChessMove2();
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
  MX_I2C2_Init();
  MX_CRC_Init();
  MX_TIM24_Init();
  MX_TIM23_Init();
  MX_TIM6_Init();
  MX_UART5_Init();
  MX_ADC1_Init();
  MX_ADC2_Init();
  MX_TIM2_Init();
  /* USER CODE BEGIN 2 */
  	MX_DMA_Init();
	HAL_GPIO_WritePin(GPIOD, GPIO_PIN_6, 1);	// LVDS EN
	HAL_GPIO_WritePin(GPIOE, GPIO_PIN_6, 1);	// Level Shifter EN

	Temp_Calibration = (110.0 - 30.0)	/ (*(unsigned short*) (0x1FF1E840) - *(unsigned short*) (0x1FF1E820));
	HAL_ADCEx_Calibration_Start(&hadc3, ADC_CALIB_OFFSET, ADC_SINGLE_ENDED);

	/*			   Encoder				*/
	AS5047U_init(&Encoder[0], &hspi3, GPIOD, &hcrc, GPIO_PIN_0,6500);
	AS5047U_init(&Encoder[1], &hspi3, GPIOD, &hcrc, GPIO_PIN_1,10100);
	AS5047U_init(&Encoder[2], &hspi3, GPIOD, &hcrc, GPIO_PIN_2,3165);
	AS5047U_init(&Encoder[3], &hspi3, GPIOD, &hcrc, GPIO_PIN_3,6970);

	HAL_Delay(50);

	/* Encoder ABI Res Setting */
//	AS5047U_Write(&Encoder[0], 0x0019, 0b00100000);
//	AS5047U_Write(&Encoder[1], 0x0019, 0b00100000);
//	AS5047U_Write(&Encoder[2], 0x0019, 0b00100000);
//	AS5047U_Write(&Encoder[3], 0x0019, 0b00100000);

	AS5047U_Write(&Encoder[0], 0x001A, 0b01000000);
	AS5047U_Write(&Encoder[1], 0x001A, 0b01000000);
	AS5047U_Write(&Encoder[2], 0x001A, 0b01000000);
	AS5047U_Write(&Encoder[3], 0x001A, 0b01000000);

	/*			Kalman Filter			*/
	Kalman_init(&Kalman[0], 5000, 0.001);
	Kalman_init(&Kalman[1], 5000, 0.001);
	Kalman_init(&Kalman[2], 5000, 0.001);
	Kalman_init(&Kalman[3], 5000, 0.001);
	Kalman_init(&Kalman[5], 5000, 0.001);

	/*			CascadeControl			*/
//	CascadeControl_init(&Control[0], 0.6, 0, 0, 15, 0.5, 10, 400);
//	CascadeControl_init(&Control[1], 0.7, 0, 0.3, 10, 0.1, 5, 430);
//	CascadeControl_init(&Control[2], 0.7, 0, 0, 10, 0.2, 0, 450);
//	CascadeControl_init(&Control[3], 0.8, 0.005, 0, 10, 0.1, 3, 470);

	CascadeControl_init(&Control[0], 0.6, 0, 0, 15, 0.5, 10, 400);
	CascadeControl_init(&Control[1], 0.7, 0, 0.3, 10, 0, 10, 410);
	CascadeControl_init(&Control[2], 0.82, 0, 0.2, 11, 0.13, 3, 445);
	CascadeControl_init(&Control[3], 0.7, 0.001, 0, 9, 0.1, 3, 500);

	/*  Power Supply Temperature Sensor */
	TPM75_init(&TempSensor, &hi2c2, 0, 0, 1);

	SetPoint_Position[0] = EncPulse2Rad_Read(&Encoder[0], 1);
	SetPoint_Position[1] = EncPulse2Rad_Read(&Encoder[1], 0);
	SetPoint_Position[2] = EncPulse2Rad_Read(&Encoder[2], 0);
	SetPoint_Position[3] = EncPulse2Rad_Read(&Encoder[3], 0);
	SetPoint_Velocity[0] = 0;
	SetPoint_Velocity[1] = 0;
	SetPoint_Velocity[2] = 0;
	SetPoint_Velocity[3] = 0;

	/*			Stepper Driver			*/
	Step_Driver_init(&Stepper[0], &htim13, TIM_CHANNEL_1, GPIOE, GPIO_PIN_0, 500000, 1);
	Step_Driver_init(&Stepper[1], &htim14, TIM_CHANNEL_1, GPIOE, GPIO_PIN_1, 500000, 0);
	Step_Driver_init(&Stepper[2], &htim15, TIM_CHANNEL_1, GPIOE, GPIO_PIN_2, 500000, 0);
	Step_Driver_init(&Stepper[3], &htim16, TIM_CHANNEL_1, GPIOE, GPIO_PIN_3, 500000, 0);

	/*			Trajectory			*/

	Test_traj_Val[0] = 0;
	Test_traj_Val[1] = 0;
	Test_traj_Val[2] = 0;
	Test_traj_Val[3] = 0;
	Traj_Flag = 0;
	t = 0;

	HAL_TIM_Base_Start_IT(&htim23);   // Start Control Timer
	HAL_UART_Receive_IT(&huart5, UART5_rxBuffer, 14);

	ChessMoveStates = Chess_idle;

  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  while (1)
  {
    /* USER CODE END WHILE */

    /* USER CODE BEGIN 3 */
		if (Test_traj) {
			Test_traj = 0;
			JMoveTaskSpace(TaskSpace_Live_Ex1, Time_Live_Ex1);
		}
		if (Test_traj2) {
			Test_traj2 = 0;
			float TaskSpace_Live_Ex1_ChangeZ[3];
			TaskSpace_Live_Ex1_ChangeZ[0] = TaskSpace_Live_Ex1[0];
			TaskSpace_Live_Ex1_ChangeZ[1] = TaskSpace_Live_Ex1[1];
			TaskSpace_Live_Ex1_ChangeZ[2] = TaskSpace_Live_Ex1[2] - 100;
			JMoveTaskSpace(TaskSpace_Live_Ex1_ChangeZ, Time_Live_Ex1);
		}

	   uint8_t tune_joint = 1;
	   if (tune_PID[0] != tune_PID[1]){
		   	tune_PID[1] = tune_PID[0];
			t = 0;
			float Joint[4] = {0,0,0,0};
			for (int i = 0; i < 4; i++) {
				Joint[i] = Control[i].PositionFeedback;
			}
			Joint[tune_joint] = tune_PID[0];
			for (int i = 0; i < 4; i++) {
				Traj_Coeff_Cal(&Traj[i], T_tune_PID, Joint[i], Control[i].PositionFeedback,0, Control[i].VelocityFeedback);
			}
			Traj_Flag = 0x0F;
	   }

		if (Contorl_Flag) {
			Control_Function();
			Contorl_Flag = 0;    // Clear Control Flag
		}

		if (Protocol_Flag) {
			Narwhal_Protocol();
			Protocol_Flag = 0;
		}

		if (HAL_GetTick() - Software_Timer_100ms >= 10) {
			Software_Timer_100ms = HAL_GetTick();
			HAL_ADC_Start_IT(&hadc3); 				//	read temperature sensor
			HAL_I2C_Mem_Read_IT(&hi2c2, 0x92, 0x00, I2C_MEMADD_SIZE_8BIT,
					Buffer_TPM75, 2);
			ChessMoveStateMachine();
		}

		if (HAL_GetTick() - Software_Timer_1s >= 1000) {		// 	Update System Status
			Software_Timer_1s = HAL_GetTick();
			HAL_GPIO_TogglePin(LED1_GPIO_Port, LED2_Pin);
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

void HAL_UART_RxCpltCallback(UART_HandleTypeDef *huart) {
	if (huart == &huart5) {
		Protocol_Flag = 1;
	}
}

void HAL_TIM_PWM_PulseFinishedCallback(TIM_HandleTypeDef *htim) {
	HAL_TIM_PWM_Stop_DMA(Neopixel.htim, TIM_CHANNEL_1);
}

void HAL_ADC_ConvCpltCallback(ADC_HandleTypeDef *hadc) {
	if (hadc == &hadc3) {
		uint32_t ADC_Output = HAL_ADC_GetValue(&hadc3);
		Temperature = ((Temperature * 9)
				+ (Temp_Calibration
						* (float) ((float) ADC_Output
								- *(unsigned short*) (0x1FF1E820)) + 30.0f))
				/ 10;
	}
}

void HAL_TIM_PeriodElapsedCallback(TIM_HandleTypeDef *htim) {
	if (htim == &htim23) {
		Contorl_Flag = 1;
	}
}

inline uint8_t CRC8(uint8_t *Data,uint8_t BufferLength){
	return HAL_CRC_Calculate(&hcrc, (uint32_t*) Data, BufferLength) ^ 0xFF;
}
inline void SentData(uint8_t range){
	UART5_txBuffer[range] = CRC8(UART5_txBuffer, range);
	HAL_UART_Transmit_IT(&huart5, UART5_txBuffer, range+1);
}

inline void Narwhal_Protocol() {
	if (UART5_rxBuffer[0] == 0xFF) {
		uint8_t CRC_Cal = CRC8(UART5_rxBuffer, 13);
		if (CRC_Cal == UART5_rxBuffer[13]) {
			UART5_txBuffer[0] = 0xFF;

			float q_Feed[4];
			float dq[4];
			float d_Task[3];
			float q_in[5] = {0,0,0,0,0};
			float task[3] = { -500, 300, 50 };
			uint16_t Temperature_Protocol = Temperature * 1000;

			/* 		Data to MCU Start	*/
			switch (UART5_rxBuffer[1]) {
			case 0xF0:
				/* Ping */
				UART5_txBuffer[2] = 0x00;
				UART5_txBuffer[1] = 0xFF;
				SentData(3);
				break;
			case 0xF1:
				/* Working Mode Set */
				UART5_txBuffer[2] = 0x01;
				UART5_txBuffer[1] = 0xFF;
				SentData(3);
				break;
			case 0xF5:
				/* GoHome */
				if (Traj_Flag == 0) {;
					t = 0;
					for (int i = 0; i < 4; i++) {
						Traj_Coeff_Cal(&Traj[i], 2, 0,
								Control[i].PositionFeedback, 0,
								Control[i].VelocityFeedback);
					}
					Traj_Flag = 0x0F;
					Chessmove_State = 0;
					UART5_txBuffer[2] = 0x00;
				} else {
					UART5_txBuffer[2] = 0x01;
				}
				UART5_txBuffer[1] = 0xFF;
				SentData(3);
				break;
			case 0xFA:
				/* Joint Jog */
				t = 0;
				for (int i = 0; i < 4; i++) {
					float JointJog = (int16_t) (((UART5_rxBuffer[2 + (2 * i)])
							<< 8) | (UART5_rxBuffer[3 + (2 * i)])) / 1000.0;
					Traj_Coeff_Cal(&Traj[i], 0.5,
							JointJog + Control[i].PositionFeedback,
							Control[i].PositionFeedback, 0,
							Control[i].VelocityFeedback);
				}
				Traj_Flag = 0x0F;
				Chessmove_State = 0;
				UART5_txBuffer[2] = 0x00;
				UART5_txBuffer[1] = 0xFF;
				SentData(3);
				break;
			case 0xFB:
				/* Cartesian Jog */
				q_Feed[0] = Control[0].PositionFeedback;
				q_Feed[1] = Control[1].PositionFeedback;
				q_Feed[2] = Control[2].PositionFeedback;
				q_Feed[3] = Control[3].PositionFeedback;
				for (int i = 0; i < 3; i++) {
					d_Task[i] = (int16_t) (((UART5_rxBuffer[2 + (2 * i)]) << 8)
							| (UART5_rxBuffer[3 + (2 * i)])) / 50.0;
				}
				IVK(q_Feed, d_Task, dq);
				t = 0;
				for (int i = 0; i < 3; i++) {
					float Setpoint = Control[i].PositionFeedback + dq[i];
					Traj_Coeff_Cal(&Traj[i], 0.5, Setpoint,
							Control[i].PositionFeedback, 0,
							Control[i].VelocityFeedback);
				}
				Traj_Flag = 0x0F;
				Chessmove_State = 0;
				UART5_txBuffer[2] = 0x00;
				UART5_txBuffer[1] = 0xFF;
				SentData(3);
				break;
			case 0xFD:
				/* Joint Set */
				t = 0;
				float Joint[4];
				float T;
				for (int i = 0; i < 4; i++) {
					Joint[i] = (int16_t) (((UART5_rxBuffer[2 + (2 * i)]) << 8)
							| (UART5_rxBuffer[3 + (2 * i)])) / 1000.0;
				}
				T = 0.5;
				for (int i = 0; i < 4; i++) {
					Traj_Coeff_Cal(&Traj[i], T, Joint[i],
							Control[i].PositionFeedback, 0,
							Control[i].VelocityFeedback);
				}
				Traj_Flag = 0x0F;
				Chessmove_State = 0;
				UART5_txBuffer[2] = 0x03;
				UART5_txBuffer[1] = 0xFF;
				SentData(3);
				break;
			case 0xFE:
				/* Cartesian Set */
				t = 0;
				for (int i = 0; i < 4; i++) {
					float JointJog = (int16_t) (((UART5_rxBuffer[2 + (2 * i)])
							<< 8) | (UART5_rxBuffer[3 + (2 * i)])) / 1000.0;
					Traj_Coeff_Cal(&Traj[i], 0.5,
							JointJog + Control[i].PositionFeedback,
							Control[i].PositionFeedback, 0,
							Control[i].VelocityFeedback);
				}
				Traj_Flag = 0x0F;
				Chessmove_State = 0;
				UART5_txBuffer[2] = 0x00;
				UART5_txBuffer[1] = 0xFF;
				SentData(3);
				break;
			case 0xFF:
				/* Chess Move */
				UART5_txBuffer[2] = 0x05;
				UART5_txBuffer[1] = 0xFF;
				SentData(3);
				break;
				/* 		Data to MCU End	*/

				/* 		Sent Data to Master Start 	*/
			case 0xA0:
				/* System Status */
				UART5_txBuffer[1] = 0xEE;
				UART5_txBuffer[2] = (uint8_t) ((Temperature_Protocol >> 8) & 0xFF);
				UART5_txBuffer[3] = (uint8_t) (Temperature_Protocol & 0xFF);
				SentData(4);
				break;
			case 0xA1:
				/* Station Encoder Position */
				UART5_txBuffer[1] = 0xEE;
				UART5_txBuffer[2] = (uint8_t) ((Encoder[0].Position >> 8) & 0xFF);
				UART5_txBuffer[3] = (uint8_t) (Encoder[0].Position & 0xFF);
				SentData(4);
				break;
			case 0xA2:
				/* Raw Joint Encoder Position */
				UART5_txBuffer[1] = 0xEE;
				for (int i = 0; i < 4; i++) {
					UART5_txBuffer[2 + (2 * i)] = (uint8_t) ((Encoder[i].Position >> 8)
							& 0xFF);
					UART5_txBuffer[3 + (2 * i)] = (uint8_t) (Encoder[i].Position & 0xFF);
				}
				SentData(12);
				break;
			case 0xAA:
				/* Joint Position */
				UART5_txBuffer[1] = 0xEE;
				for (int i = 0; i < 4; i++) {
					int16_t Buf = (int16_t) (Control[i].PositionFeedback
							* 1000.0f);
					UART5_txBuffer[2 + (2 * i)] = (uint8_t) ((Buf >> 8) & 0xFF);
					UART5_txBuffer[3 + (2 * i)] = (uint8_t) (Buf & 0xFF);
				}
				UART5_txBuffer[10] = 0;
				UART5_txBuffer[11] = 0;
				SentData(12);
				break;
			case 0xAB:
				/* Task Space Position */
				UART5_txBuffer[1] = 0xEE;
				/*	Forward Kinematics */

				for (int i = 0; i < 4; i++) {
					q_in[i] = Control[i].PositionFeedback;
				}
				FPK(q_in, 269.0f, task);

				for (int i = 0; i < 3; i++) {
					int16_t Buf = (int16_t) (task[i] * 10.0f);
					UART5_txBuffer[2 + (2 * i)] = (uint8_t) ((Buf >> 8) & 0xFF);
					UART5_txBuffer[3 + (2 * i)] = (uint8_t) (Buf & 0xFF);
				}
				SentData(8);
				break;
			case 0xAF:
				/* UI Feedback */
				UART5_txBuffer[1] = 0xEE;
				/*	Forward Kinematics */
				for (int i = 0; i < 4; i++) {
					q_in[i] = Control[i].PositionFeedback;
				}
				FPK(q_in, 269.0f, task);

				/* Station Encoder */
				UART5_txBuffer[2] = (uint8_t) ((Encoder[0].Position >> 8) & 0xFF);
				UART5_txBuffer[3] = (uint8_t) (Encoder[0].Position & 0xFF);

				/* Temp */
				UART5_txBuffer[4] = (uint8_t) ((Temperature_Protocol >> 8) & 0xFF);
				UART5_txBuffer[5] = (uint8_t) (Temperature_Protocol & 0xFF);
				UART5_txBuffer[6] = (uint8_t) ((Temperature_Protocol >> 8) & 0xFF);
				UART5_txBuffer[7] = (uint8_t) (Temperature_Protocol & 0xFF);

				/* Joint Current */
				for (int i = 0; i < 4; i++) {
					int16_t Buf = (int16_t) (Control[i].PositionFeedback
							* 1000.0f);
					UART5_txBuffer[8 + (2 * i)] = (uint8_t) ((Buf >> 8) & 0xFF);
					UART5_txBuffer[9 + (2 * i)] = (uint8_t) (Buf & 0xFF);
				}
				UART5_txBuffer[16] = 0;
				UART5_txBuffer[17] = 0;

				/* Joint Set point*/
				for (int i = 0; i < 4; i++) {
					int16_t Buf = (int16_t) (SetPoint_Position[i] * 1000.0f);
					UART5_txBuffer[18 + (2 * i)] = (uint8_t) ((Buf >> 8) & 0xFF);
					UART5_txBuffer[19 + (2 * i)] = (uint8_t) (Buf & 0xFF);
				}
				UART5_txBuffer[26] = 0;
				UART5_txBuffer[27] = 0;

				for (int i = 0; i < 3; i++) {
					int16_t Buf = (int16_t) (task[i] * 10.0f);
					UART5_txBuffer[28 + (2 * i)] = (uint8_t) ((Buf >> 8) & 0xFF);
					UART5_txBuffer[29 + (2 * i)] = (uint8_t) (Buf & 0xFF);
				}
				SentData(34);
				break;
			default:
				UART5_txBuffer[2] = 0xFF;
				SentData(3);
				/* 		Sent Data to Master End 	*/
			}
		} else {
			/*		CRC Error		*/
			UART5_txBuffer[1] = 0xCC;
			SentData(3);
		}
	} else {
		/*		Header Error	*/
		UART5_txBuffer[1] = 0xAA;
		SentData(3);
	}
	HAL_UART_Receive_IT(&huart5, UART5_rxBuffer, 14);
}

inline void JointSpaceTraj(float Task2Move[3],float Time2Move){
	float gamma[3] = {1,1,-1};
	float q_inv[4];
	IPK(gamma, Task2Move, q_inv);
	t = 0;
	for (int i = 0; i < 4; i++) {
		Traj_Coeff_Cal(&Traj[i], Time2Move, q_inv[i],
				Control[i].PositionFeedback,  0,
				Control[i].VelocityFeedback);
	}
	Traj_Flag = 0x0F;
	Chessmove_State = 0;
}

inline void Control_Function() {
	/***** Encoder Read *****/
	float J1, J2, J3, J4;
	J1 = EncPulse2Rad_Read(&Encoder[0], 1);
	J2 = EncPulse2Rad_Read(&Encoder[1], 0);
	J3 = EncPulse2Rad_Read(&Encoder[2], 0);
	J4 = EncPulse2Rad_Read(&Encoder[3], 0);

	/***** Joint Space SetPoint Gen *****/
	if (Chessmove_State == 0) {
		if (Traj_Flag & 0x0F) {
			float traj_t_set[5];
			traj_t_set[0] = t;
			traj_t_set[1] = t * t;
			traj_t_set[2] = traj_t_set[1] * t;
			traj_t_set[3] = traj_t_set[2] * t;
			traj_t_set[4] = traj_t_set[3] * t;
			for (int i = 0; i < 4; i++) {
				if (Traj_Flag & (0x01 << i)) {
					TrajFollow(&Traj[i], traj_t_set, &SetPoint_Position[i],
							&SetPoint_Velocity[i]);
					if (t >= Traj[i].T) {
						Traj_Flag &= ((0x01 << i) ^ 0xFF);
					}
				}
			}
			t += delta_t;
		}
	}

	else if (Chessmove_State == 1) {
		if (Traj_Flag & 0x0F) {
			float gamma[3] = { 1, 1, -1 };
			float Chi_t[3];
			float ChiDot_t[3];

			//*********** Chi Output ************//
			float SetPointPosition[4];
			float SetPointVelocity[4];

			float traj_t_set[5];
			traj_t_set[0] = t;
			traj_t_set[1] = t * t;
			traj_t_set[2] = traj_t_set[1] * t;
			traj_t_set[3] = traj_t_set[2] * t;
			traj_t_set[4] = traj_t_set[3] * t;
			for (int i = 4; i < 7; i++) {
				TrajFollow(&Traj[i], traj_t_set, &Chi_t[i], &ChiDot_t[i]);
				if (t >= Traj[i].T) {
					Traj_Flag &= ((0x01 << i) ^ 0xFF);
				}
			}
			IPK(gamma, Chi_t, SetPointPosition);
			IVK(SetPointPosition, ChiDot_t, SetPointVelocity);
			for (int i = 0; i < 4; i++) {
				TrajFollow(&Traj[i], traj_t_set, &SetPointPosition[i],
						&SetPointVelocity[i]);
			}
			t += delta_t;
		}
	}

	CascadeControl(&Control[0], &Kalman[0], J1, SetPoint_Position[0], SetPoint_Velocity[0]);
	CascadeControl(&Control[1], &Kalman[1], J2, SetPoint_Position[1], SetPoint_Velocity[1]);
	CascadeControl(&Control[2], &Kalman[2], J3, SetPoint_Position[2], SetPoint_Velocity[2]);
	CascadeControl(&Control[3], &Kalman[3], J4, SetPoint_Position[3], SetPoint_Velocity[3]);

	Step_Driver(&Stepper[0], Control[0].Output);
	Step_Driver(&Stepper[1], Control[1].Output);
	Step_Driver(&Stepper[2], Control[2].Output);
	Step_Driver(&Stepper[3], Control[3].Output);
}

inline void JMoveTaskSpace(float Task2Go[3], float Time2Move){
	float gamma[3] = { 1, 1, -1 };
	float q_inv[4];
	IPK(gamma, Task2Go, q_inv);
	t = 0;
	for (int i = 0; i < 4; i++) {
		Traj_Coeff_Cal(&Traj[i], Time2Move, q_inv[i], Control[i].PositionFeedback, 0,
				Control[i].VelocityFeedback);
	}
	Traj_Flag = 0x0F;
	Chessmove_State = 0;
}

inline void LMoveTaskSpace(float Task2Go[3], float Time2Move){
	float q_Feed[4];
	float qv_Feed[4];
	float Pne[3];
	float vPne[3];
	q_Feed[0] = Control[0].PositionFeedback;
	q_Feed[1] = Control[1].PositionFeedback;
	q_Feed[2] = Control[2].PositionFeedback;
	q_Feed[3] = Control[3].PositionFeedback;
	qv_Feed[0] = Control[0].VelocityFeedback;
	qv_Feed[1] = Control[1].VelocityFeedback;
	qv_Feed[2] = Control[2].VelocityFeedback;
	qv_Feed[3] = Control[3].VelocityFeedback;

	FPK(q_Feed, 269.0f, Pne);
	FVK(q_Feed, qv_Feed, 269.0f, vPne);
	t = 0;
	for (int i = 4; i < 7; i++) {
		Traj_Coeff_Cal(&Traj[i], Time2Move, Task2Go[i], Pne[i], 0, vPne[i]);
	}
	Traj_Flag = 0x0F;
	Chessmove_State = 1; // Change to TaskSpace Traj
}

inline void ChessNotMovePathWay(uint8_t Index2Move, float Z_Offset, uint8_t IsJMove) {
	/***** Encoder Read *****/
	float BaseEnc;
	float PositionXY[2];

	/***** Base Encoder Read *****/
//	BaseEnc = BaseENCRead();   //?????????????????????????????????????????????????????????
	BaseEnc = 0;
	ChessPose(Index2Move, BaseEnc, PositionXY);
	float TaskSpace2Go[3];
	TaskSpace2Go[0] = PositionXY[0];
	TaskSpace2Go[1] = PositionXY[1];
	TaskSpace2Go[2] = Z_Offset;
	if (IsJMove){
		JMoveTaskSpace(TaskSpace2Go, 2);
	}
	else{
		JMoveTaskSpace(TaskSpace2Go, 2);
	}
}

void ChessMoveStateMachine() {
	static uint8_t ChangeState = 0;
	switch (ChessMoveStates) {
	case Chess_idle:
		if (Chess_Move_Start_Flag) {
			ChessMoveStates = Move_2_Start_Top_Point;
		}
		break;
	case Move_2_Start_Top_Point:
		if (ChangeState) {
			ChessNotMovePathWay(ChessIndex[0], Z_TopOffset, 1);
			ChangeState = 0;
		}
		if (!Traj_Flag) {
			ChessMoveStates = Move_2_Start_Grip_Point;
			ChangeState = 1;
		}
		break;
	case Move_2_Start_Grip_Point:
		if (ChangeState) {
			ChessNotMovePathWay(ChessIndex[0], Z_2GripOffset, 0);
			ChangeState = 0;
		}
		if (!Traj_Flag) {
			ChessMoveStates = Move_2_Start_Point_and_Griping;
			ChangeState = 1;
		}
		break;
	case Move_2_Start_Point_and_Griping:
		// Grip Chess

		ChessMoveStates = Back_2_Start_Top_Point;
		break;
	case Back_2_Start_Top_Point:
		if (ChangeState) {
			ChessNotMovePathWay(ChessIndex[0], Z_TopOffset, 0);
			ChangeState = 0;
		}
		if (!Traj_Flag) {
			ChessMoveStates = Move_2_End_Top_Point;
			ChangeState = 1;
		}
		break;

		// Finish To Grip Chess

	case Move_2_End_Top_Point:
		if (ChangeState) {
			ChessNotMovePathWay(ChessIndex[1], Z_TopOffset, 1);
			ChangeState = 0;
		}
		if (!Traj_Flag) {
			ChessMoveStates = Move_2_End_Ungrip_Point;
			ChangeState = 1;
		}
		break;
	case Move_2_End_Ungrip_Point:
		if (ChangeState) {
			ChessNotMovePathWay(ChessIndex[1], Z_2GripOffset, 0);
			ChangeState = 0;
		}
		if (!Traj_Flag) {
			ChessMoveStates = Move_2_End_Point_and_Ungriping;
			ChangeState = 1;
		}
		break;
	case Move_2_End_Point_and_Ungriping:
		// Grip Chess

		ChessMoveStates = Back_2_End_Top_Point;
		break;
	case Back_2_End_Top_Point:
		if (ChangeState) {
			ChessNotMovePathWay(ChessIndex[1], Z_TopOffset, 0);
			ChangeState = 0;
		}
		if (!Traj_Flag) {
			ChessMoveStates = ChessMove_Finish;
			ChangeState = 1;
		}
		break;
	case ChessMove_Finish:
		Chess_Move_Start_Flag = 0;
		ChessMoveStates = Chess_idle;
		break;
	default:
		break;
	}
}

//void Chess_Tracker(uint8_t Chess_Index, float Chess_Theta, float Chess_Omega){
//	/*
//	 * Offset2Center = robot to chess board center
//	 */
//	float Offset2Center[2] = {300,0};
//	/*
//	 * Output
//	 *
//	 */
//	float ChessPosition[2];
//	float ChessVelocity[2];
//
//	float ChessRadius;
//	float TzOnly = 3;
//    uint8_t X = Chess_Index%8;
//    uint8_t N = Chess_Index/8;
//
//    ChessPose(X, N, Chess_Theta, ChessPosition);
//    FindR(Offset2Center, ChessPosition, &ChessRadius);
//    FindXYSpeed(Chess_Theta, Chess_Omega, ChessRadius, ChessVelocity);
//
//    /*
//	 * Traj Z Axis
//	 */
//    for (int i = 0; i < 4; i++) {
//    		q_in[i] = Control[i].PositionFeedback;
//    	}
//    FPK(q_in, 269.0f, EndEffectorNow);
//	Traj_Coeff_Cal(&Traj[4], TzOnly, Z_Top_Offset,
//			      EndEffectorNow, 0, 0); /* Cal C for Trajz*/
//	Trajz_Flag = 1;
//	if (Chessmove_State == 1 && Trajz_Flag == 1) { /* Let do while in check position state and have Cal Coeff Z*/
//		float traj_t_set[5];
//		float TaskZ_Position = 0;
//		float TaskZ_Velocity = 0;
//		traj_t_set[0] = t;
//		traj_t_set[1] = t * t;
//		traj_t_set[2] = traj_t_set[1] * t;
//		traj_t_set[3] = traj_t_set[2] * t;
//		traj_t_set[4] = traj_t_set[3] * t;
//		TrajFollow(&Traj[4], traj_t_set, TaskZ_Position,
//				   TaskZ_Velocity);
//	}
//	if (t >= Traj[4].T) { /* Trajz finished */
//		Trajz_Flag = 0;
//	}
//	t += 0.005;
//
//    // Transfer to Joint
//	float EndEffectorgoal[3];
//	EndEffectorgoal[0] = ChessPosition[0];
//	EndEffectorgoal[1] = ChessPosition[1];
//	EndEffectorgoal[2] = TaskZ_Position;
//	float gamma[3] = { 1, 1, -1 };
//	IPK(gamma, EndEffectorgoal, SetPoint_Position);
//	d_Task[0] = ChessVelocity[0];
//	d_Task[1] = ChessVelocity[1];
//	d_Task[2] = TaskZ_Velocity;
//	IVK(SetPoint_Position, d_Task, SetPoint_Velocity);
//
//}

//void ChessMove1(uint8_t StartIndex, uint8_t EndIndex){
//	float Z_Top_Offset = 200;
//
//	float EndEffectorNow[3];
//	float EndEffectorAtChess[2];
//	float q_in[5];
//
//	for (int i = 0; i < 4; i++) {
//		q_in[i] = Control[i].PositionFeedback;
//	}
//	ChessPose(StartIndex, Kalman[4].x1, EndEffectorAtChess);
//	FPK(q_in, 269.0f, EndEffectorNow);
//	float DeltaX = EndEffectorNow[0] - EndEffectorAtChess[0];
//	float DeltaY = EndEffectorNow[1] - EndEffectorAtChess[1];
//	float DeltaZ = EndEffectorNow[2] - Z_Top_Offset;
//	float Distance = sqrt(
//			(DeltaX * DeltaX) + (DeltaY * DeltaY) + (DeltaZ * DeltaZ));
//	float T2Move = (Distance / 150.0f) + 1;
//
////	/*
////	 * Traj Z Axis
////	 */
////
////	Traj_Coeff_Cal(&Traj[4], T2Move, Z_Top_Offset,
////			EndEffectorNow, 0, 0); /* Cal C for Trajz*/
////	Trajz_Flag = 1;
////	if (Chessmove_State == 1 && Trajz_Flag == 1) { /* Let do while in check position state and have Cal Coeff Z*/
////		float traj_t_set[5];
////		traj_t_set[0] = t;
////		traj_t_set[1] = t * t;
////		traj_t_set[2] = traj_t_set[1] * t;
////		traj_t_set[3] = traj_t_set[2] * t;
////		traj_t_set[4] = traj_t_set[3] * t;
////		TrajFollow(&Traj[4], traj_t_set, &SetPoint_Position[4], /* SetPoint_Position[4] valueof z axis not related to control */
////								&SetPoint_Velocity[4]);
////	}
////	if (t >= Traj[4].T) { /* Trajz finished */
////		Trajz_Flag = 0;
////	}
////	t += 0.005;
//
//	/*
//	 * Traj 2 Move XY
//	 */
//
//	float EndEffectorTarget[3];
//	EndEffectorTarget[2] = Z_Top_Offset;
//	ChessPose(StartIndex, Kalman[4].x1 + (T2Move * (Kalman[4].x2)) , EndEffectorTarget);
//
//	float gamma[3] = { 1, 1, -1 };
//	float q_inv[4];
//	IPK(gamma, EndEffectorTarget, q_inv);
//	t = 0;
//	for (int i = 0; i < 4; i++) {
//		Traj_Coeff_Cal(&Traj[i], T2Move, q_inv[i],
//				Control[i].PositionFeedback, 0, Control[i].VelocityFeedback);
//	}
//	Traj_Flag = 0x0F;
//	Chessmove_State = 0;
//
//}
//
//uint8_t ChessMove2() {
//	float EndEffectorNow[3];
//	float q_now[5];
//	for (int i = 0; i < 4; i++) {
//		q_now[i] = Control[i].PositionFeedback;
//	}
//	FPK(q_now, 269.0f, EndEffectorNow);
//	uint8_t count_ready = 0;
//	for (int i = 0; i < 4; i++) {
//		if (fabs(EndEffectorNow[i] - EndEffectorTarget[i]) <= 0.01) {
//			count_ready += 1;
//		}
//		if (count_ready == 3) {  	// Finish Task
//			return 1;
//		} else {						// Move Not Finish
//			return 0;
//		}
//	}
//	return 0;
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
