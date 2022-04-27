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

Kinematics_StatusTypeDef IPK(double gammabar[3], double chi[4], double q[5]);
Kinematics_StatusTypeDef IVK(double q[4], double chi_dot[3], double qv[4]);
void FPK(double q[5], double l3, double Pne[3]);
void FVK(double q[4], double qd[4], double l3, double twist[3]);

void ChessPose(uint8_t Chess_Index, double Chess_Theta, double ChessPosition[2]);
void FindR(double Offset2Center[2], double chessPosition[2], double *ChessRadius);
void FindXYSpeed(double Chess_Theta, double Chess_Omaga, double ChessRadius, double SpeedXY[2]);

#endif /* INC_KINEMATICS_H_ */
