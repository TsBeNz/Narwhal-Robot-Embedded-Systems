/*
 * kinematics.c
 *
 *  Created on: Jan 24, 2022
 *      Author: matas manawakul
 *      	  : Thansak Pongpaket
 */

#include "kinematics.h"


/**************** Input *********************
 *
 * gammabar[3]		<--- Robot Pos [1,1,-1]
 * Chi[3]			<--- TaskSpace Position [x,y,z]
 *
 **************** Output ********************
 * q[4]				---> JointSpace Position
 *
 */
Kinematics_StatusTypeDef IKnarwhale(float gammabar[3], float chi[3], float q[4]) {
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
		return Kinematics_OK;
	} else {
		return Error_Link_length;
	}
}



/************* Input *****************
 *
 * q[3]		<---	Joint Configuration (J. 1-3)
 * chi_dot	<---	TaskSpace Velocity 	(x,y,z)
 *
 ************** Output ****************
 *
 * qv[4]	--->	Joint Velocity 		(J. 1-4)
 *
 */

//Kinematics_StatusTypeDef IVK(float q[3], float chi_dot[3], float qv[4])

Kinematics_StatusTypeDef IVK(float q[4], float chi_dot[3], float qv[4])
{
  float Jv4[9];
  float Jv4_tmp;
  float Jv4_tmp_tmp;
  float b_Jv4_tmp;
  float c_Jv4_tmp;
  float qvbar_idx_1;
  float qvbar_idx_2;
  int r1;
  int r2;
  int rtemp;
  qvbar_idx_1 = q[1] + q[2];
  qvbar_idx_2 = sin(qvbar_idx_1);
  Jv4_tmp = cos(q[0]);
  b_Jv4_tmp = sin(q[0]);
  qvbar_idx_1 = 380.0 * cos(qvbar_idx_1);
  Jv4_tmp_tmp = 380.0 * sin(q[1]);
  c_Jv4_tmp = (qvbar_idx_1 + 20.0) - Jv4_tmp_tmp;
  Jv4[0] = -b_Jv4_tmp * c_Jv4_tmp;
  Jv4[3] = -Jv4_tmp * (380.0 * qvbar_idx_2 + 380.0 * cos(q[1]));
  Jv4[6] = -380.0 * qvbar_idx_2 * Jv4_tmp;
  Jv4[1] = Jv4_tmp * c_Jv4_tmp;
  Jv4[4] = -sin(q[0]) * (380.0 * sin(q[1] + q[2]) + 380.0 * cos(q[1]));
  Jv4[7] = -380.0 * sin(q[1] + q[2]) * b_Jv4_tmp;
  Jv4[2] = 0.0;
  Jv4[5] = qvbar_idx_1 - Jv4_tmp_tmp;
  Jv4[8] = qvbar_idx_1;
  r1 = 0;
  r2 = 1;
  rtemp = 2;
  if (fabs(Jv4[1]) > fabs(Jv4[0])) {
    r1 = 1;
    r2 = 0;
  }
  Jv4[r2] /= Jv4[r1];
  Jv4[2] = 0.0 / Jv4[r1];
  Jv4[r2 + 3] -= Jv4[r2] * Jv4[r1 + 3];
  Jv4[5] -= Jv4[2] * Jv4[r1 + 3];
  Jv4[r2 + 6] -= Jv4[r2] * Jv4[r1 + 6];
  Jv4[8] -= Jv4[2] * Jv4[r1 + 6];
  if (fabs(Jv4[5]) > fabs(Jv4[r2 + 3])) {
    rtemp = r2;
    r2 = 2;
  }
  Jv4[rtemp + 3] /= Jv4[r2 + 3];
  Jv4[rtemp + 6] -= Jv4[rtemp + 3] * Jv4[r2 + 6];
  qvbar_idx_1 = chi_dot[r2] - chi_dot[r1] * Jv4[r2];
  qvbar_idx_2 = ((chi_dot[rtemp] - chi_dot[r1] * Jv4[rtemp]) -
                 qvbar_idx_1 * Jv4[rtemp + 3]) /
                Jv4[rtemp + 6];
  qvbar_idx_1 -= qvbar_idx_2 * Jv4[r2 + 6];
  qvbar_idx_1 /= Jv4[r2 + 3];
  qv[0] =
      ((chi_dot[r1] - qvbar_idx_2 * Jv4[r1 + 6]) - qvbar_idx_1 * Jv4[r1 + 3]) /
      Jv4[r1];
  qv[1] = qvbar_idx_1;
  qv[2] = qvbar_idx_2;
  qv[3] = -qvbar_idx_1 - qvbar_idx_2;
  return Kinematics_OK;
}
