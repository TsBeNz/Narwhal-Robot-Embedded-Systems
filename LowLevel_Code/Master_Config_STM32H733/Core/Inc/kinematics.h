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


typedef enum
{
  Kinematics_OK       	= 0x00,
  Error_Link_length    	= 0x01,
  Singularity     		= 0x02,
} Kinematics_StatusTypeDef;


/*
 * Function
 */

Kinematics_StatusTypeDef IKnarwhale(float gammabar[3], float chi[4], float q[5]);
Kinematics_StatusTypeDef IVK(float q[3], float chi_dot[3], float qv[4]);
#endif /* INC_KINEMATICS_H_ */
