/*
 * kinematics.h
 *
 *  Created on: Jan 20, 2022
 *      Author: matas
 */

#ifndef INC_KINEMATICS_H_
#define INC_KINEMATICS_H_

// include stm32h7 driver
#include "stm32h7xx.h"

/*
 * DEFINES
 */
#define h1 275.99
#define h2 380
#define l1 20.01
#define l2 380
#define l3 235

/*
 * Function
 */
void IVK(float v_x, float v_y, float v_z, float v_pitch, float v_yaw, float *q, float *qv);

#endif /* INC_KINEMATICS_H_ */
