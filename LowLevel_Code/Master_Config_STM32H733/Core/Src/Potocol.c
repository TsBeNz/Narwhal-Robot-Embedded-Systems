/*
 * Potocol.c
 *
 *  Created on: Feb 17, 2022
 *      Author: thans
 */


#include "Protocol.h"
#include "usart.h"

//void HAL_UART_RxCpltCallback(UART_HandleTypeDef *huart) {
//	if (huart == &huart5) {
//		HAL_UART_Transmit_IT(&huart5, UART5_rxBuffer, 14);
//		HAL_UART_Receive_IT(&huart5, UART5_rxBuffer, 14);
//	} else if (huart == &huart3) {
//		HAL_UART_Transmit(&huart3, UART5_rxBuffer, 14, 100);
//		HAL_UART_Receive_IT(&huart3, UART5_rxBuffer, 14);
//	}
//}
