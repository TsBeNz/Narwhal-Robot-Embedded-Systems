/*
 * pid.h
 *
 *  Created on: Jan 27, 2022
 *      Author: matas
 */

#ifndef INC_CASCADE_PIDCONTROL_H_
#define INC_CASCADE_PIDCONTROL_H_

// include stm32h7 driver
#include "stm32h7xx.h"

float PID_Vform(float c, float k_p, float k_i, float k_d, float setpoint, float feedback, float *error_prevk, float *error_pprevk );
float Cascade_pid();
#endif /* INC_CASCADE_PIDCONTROL_H_ */
