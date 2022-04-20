/*
 * kinematics.h
 *
 *  Created on: Jan 24, 2022
 *      Author: matas manawakul
 *      	  : Thansak Pongpaket
 */

#ifndef INC_KINEMATICS_H_
#define INC_KINEMATICS_H_

// include stm32h7 driver
#include "stm32h7xx.h"
#include "math.h"
#define PI               3.14159265358979f


typedef enum
{
  Kinematics_OK       	= 0x00,
  Error_Link_length    	= 0x01,
  Singularity     		= 0x02,
} Kinematics_StatusTypeDef;


/*
 * Function
 */

Kinematics_StatusTypeDef IPK(float gammabar[3], float chi[4], float q[5]);
Kinematics_StatusTypeDef IVK(float q[4], float chi_dot[3], float qv[4]);
void FPK(float q[5], float l3, float Pne[3]);
void FVK(float q[4], float qd[4], float l3, float twist[3]);

void ChessPose(uint8_t Chess_Index, float Chess_Theta, float ChessPosition[2]);
void FindR(float Offset2Center[2], float chessPosition[2], float *ChessRadius);
void FindXYSpeed(float Chess_Theta, float Chess_Omaga, float ChessRadius, float SpeedXY[2]);

#endif /* INC_KINEMATICS_H_ */
