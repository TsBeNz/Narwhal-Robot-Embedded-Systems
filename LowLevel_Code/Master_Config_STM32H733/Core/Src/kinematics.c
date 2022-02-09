/*
 * kinematics.c
 *
 *  Created on: Jan 24, 2022
 *      Author: matas manawakul
 *      	  : Thansak Pongpaket
 */

#include "kinematics.h"

Kinematics_StatusTypeDef IKnarwhale(float gammabar[3], float chi[4], float q[5]) {
	float c2;
	float q2;
	float q3;
	float s2;
	float x24;
	x24 = gammabar[1] * sqrt(chi[0] * chi[0] + chi[1] * chi[1]) - 20.0;
	c2 = x24 * x24
			+ ((chi[2] + 268.23) - 295.89) * ((chi[2] + 268.23) - 295.89);
	s2 = sqrt(c2);
	if ((s2 <= 760.0) && (s2 >= 0.0)) {
		c2 = ((c2 - 144400.0) - 144400.0) / 288800.0;
		s2 = gammabar[2] * sqrt(1.0 - c2 * c2);
		q2 = (atan2((chi[2] + 268.23) - 295.89, x24)
				- atan2(380.0 * s2, 380.0 * c2 + 380.0)) - 1.5707963267948966;
		q3 = atan2(s2, c2) + 1.5707963267948966;
		q[0] = atan2(gammabar[0] * chi[1], gammabar[0] * chi[0]);
		q[1] = q2;
		q[2] = q3;
		q[3] = -q2 - q3;
		q[4] = chi[3];
		return Kinematics_OK;
	} else {
		return Error_Link_length;
	}
}

void IVK(float v_x, float v_y, float v_z, float v_pitch, float v_yaw, float *q,	float *qv) {
	float c1 = cos(*q);
	float s1 = sin(*q);
	float c2 = cos(*(q + 1));
	float s2 = sin(*(q + 1));
	float c23 = cos(*(q + 1) + *(q + 2));
	float s23 = sin(*(q + 1) + *(q + 2));
	float c234 = cos(*(q + 1) + *(q + 2) + *(q + 3));
	float s234 = sin(*(q + 1) + *(q + 2) + *(q + 3));
	float l2c23 = 380.0f * c23;
	float l2s23 = 380.0f * s23;
	float l3c234 = 235.0f * c234;
	float l3s234 = 235.0f * s234;
	*qv = -v_y - v_z - v_pitch;
	*(qv + 1) = v_yaw;
	*(qv + 2) = v_pitch * l3c234 * c1
			- v_y * (c1 * (l2s23 + 380.0f * c2) - l3c234 * c1)
			- v_z * c1 * (l2s23 - l3c234)
			- v_x * (s1 * (20.01f + l2c23 - 380.0f * s2) + l3s234 * s1);
	*(qv + 3) = v_x * (c1 * (20.01f + l2c23 - 380.0f * s2) + l3s234 * c1)
			- v_y * (s1 * (l2s23 + 380.0f * c2) - l3c234 * s1)
			- v_z * s1 * (l2s23 - l3c234) + v_pitch * l3c234 * s1;
	*(qv + 4) = v_y * (l2c23 - 380.0f * s2 + l3s234) + v_z * (l2c23 + l3s234)
			+ (v_pitch * l3s234);
}

