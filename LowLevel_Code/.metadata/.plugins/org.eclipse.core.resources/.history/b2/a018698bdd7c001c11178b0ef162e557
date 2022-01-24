/*
 * kinematics.c
 *
 *  Created on: Jan 20, 2022
 *      Author: matas manawakul
 */

#include "kinematics.h"
void IVK(float v_x, float v_y, float v_z, float v_pitch, float v_yaw, float *q, float *qv){
	float c1 = cos(*q);
	float s1 = sin(*q);
	float c2 = cos(*(q+1));
	float s2 = sin(*(q+1));
	float c23 = cos(*(q+1)+*(q+2));
	float s23 = sin(*(q+1)+*(q+2));
	float c234 = cos(*(q+1)+*(q+2)+*(q+3));
	float s234 = sin(*(q+1)+*(q+2)+*(q+3));
	float l2c23 = l2*c23;
	float l2s23 = l2*s23;
	float l3c234 = l3*c234;
	float l3s234 = l3*s234;
	*qv = -v_y-v_z-v_pitch;
	*(qv+1) = v_yaw;
	*(qv+2) = v_pitch*l3c234*c1 - v_y*(c1*(l2s23 + h2*c2) - l3c234*c1) - v_z*c1*(l2s23 - l3c234) - v_x*(s1*(l1 + l2c23 - h2*s2) + l3s234*s1);
	*(qv+3) = v_x*(c1*(l1 + l2c23 - h2*s2) + l3s234*c1) - v_y*(s1*(l2s23 + h2*c2) - l3c234*s1) - v_z*s1*(l2s23 - l3c234) + v_pitch*l3c234*s1;
	*(qv+4) = v_y*(l2c23 - h2*s2 + l3s234) + v_z*(l2c23 + l3s234) + (v_pitch*l3s234);
}
