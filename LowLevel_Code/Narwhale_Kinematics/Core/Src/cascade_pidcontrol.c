/*
 * pid.c
 *
 *  Created on: Jan 27, 2022
 *      Author: matas
 */
#include <cascade_pidcontrol.h>

float PID_Vform(float c, float k_p, float k_i, float k_d, float setpoint, float feedback, float *error_prevk, float *error_pprevk ){
	float error_k = setpoint - feedback;
	*error_prevk = error_k;
	*error_pprevk = *error_prevk;
	float delta_error = error_k - *error_prevk;
	float y = c*setpoint+ k_p*delta_error+ k_i*error_k+ k_d*(error_k- (2*(*error_prevk))+ *error_pprevk);

	return y;
}



