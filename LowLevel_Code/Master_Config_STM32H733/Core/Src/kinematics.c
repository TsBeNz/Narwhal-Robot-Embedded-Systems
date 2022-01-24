/*
 * kinematics.c
 *
 *  Created on: Jan 20, 2022
 *      Author: matas manawakul
 */

#include "kinematics.h"


void IPK(float x, float y, float z, float pitch, float yaw, float *q) {
	*q = atan2(y, x);
	//define part 1
	float s_pitch = sin(pitch);
	float c_pitch = cos(pitch);
	float x24 = (x - 235.0f * s_pitch) - 20.01f;
	float z24 = (z - 235.0f * c_pitch) - 275.99f;
	float x24_z24_pow2 = ((x24 * x24) + (z24 * z24));

	if (sqrt(x24_z24_pow2) <= (760)) {
		float s3 = (x24_z24_pow2 - 288800.0f) / (288800.0f);
		float c3 = sqrt(1 - (s3 * s3));
		*(q+1) = atan2((-(380.0f + (380.0f * s3)) * x24) + (380.0f * c3 * z24) , (380.0f * c3 * x24) + ((380.0f * s3) + 380.0f));
		*(q+2) = atan2(s3, c3);

		//define part 2
		float s1 = sin(*q);
		float c1 = cos(*q);
		float s23 = sin(*(q+1) + *(q+2));
		float c23 = cos(*(q+1) + *(q+2));
		float s_yaw = sin(yaw);
		float c_yaw = cos(yaw);
		float c_yaw_s_pitch = c_yaw*s_pitch;
		float s1_s_yaw = s1*s_yaw;
		float c_pitch_c1_c_yaw = c_pitch*c1*c_yaw;

		float r11 = (s23*c_yaw_s_pitch) + (c23*s1_s_yaw) + (c23*c_pitch_c1_c_yaw);
		float r21 = (c23*c_yaw_s_pitch) - (s23*s1_s_yaw) - (s23*c_pitch_c1_c_yaw);
		float r31 = (c_pitch*c_yaw*s1) - (c1*s_yaw);
		float c5 = sqrt((r11 * r11) + (r21 * r21));
		*(q+3) = atan2(r21, r11);
		*(q+4) = atan2(r31, c5);

	}
	else{
		*(q) = 0;
		*(q+1) = 0;
		*(q+2) = 0;
		*(q+3) = 0;
		*(q+4) = 0;
	}
}

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

