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
#define h1 275.99f
#define h2 380.0f
#define l1 20.01f
#define l2 380.0f
#define l3 235.0f

/*
 * Function
 */

void IPK(float x, float y, float z, float pitch, float yaw, float *q);
void IVK(float v_x, float v_y, float v_z, float v_pitch, float v_yaw, float *q, float *qv);

#endif /* INC_KINEMATICS_H_ */
