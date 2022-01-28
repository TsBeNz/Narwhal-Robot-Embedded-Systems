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

/*
 * Function
 */

void IPK(float x, float y, float z, float pitch, float yaw, float *q);
void IVK(float v_x, float v_y, float v_z, float v_pitch, float v_yaw, float *q, float *qv);

#endif /* INC_KINEMATICS_H_ */
