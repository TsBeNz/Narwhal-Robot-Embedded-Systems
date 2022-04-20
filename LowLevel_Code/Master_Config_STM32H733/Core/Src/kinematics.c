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
Kinematics_StatusTypeDef IPK(float gammabar[3], float chi[3], float q[4]) {
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


/* Function Definitions */
/*
 * Arguments    : const double q[5]
 *                double l3
 *                double Pne[3]
 * Return Type  : void
 */
void FPK(float q[5], float l3, float Pne[3]) {
	float Pne_tmp;
	float b_Pne_tmp;
	float c_Pne_tmp;
	float d_Pne_tmp;
	float e_Pne_tmp;
	float f_Pne_tmp;
	float g_Pne_tmp;
	float h_Pne_tmp;
	/* offset */
	Pne_tmp = cos(q[0]);
	b_Pne_tmp = sin(q[2]);
	c_Pne_tmp = cos(q[2]);
	d_Pne_tmp = sin(q[1]);
	e_Pne_tmp = cos(q[1]);
	f_Pne_tmp = sin(q[0]);
	g_Pne_tmp = cos(q[3]);
	h_Pne_tmp = sin(q[3]);
	Pne[0] =
			((380.0
					* (Pne_tmp * e_Pne_tmp * c_Pne_tmp
							- Pne_tmp * d_Pne_tmp * b_Pne_tmp)
					+ l3
							* (g_Pne_tmp
									* (cos(q[0]) * cos(q[1]) * b_Pne_tmp
											+ Pne_tmp * c_Pne_tmp * d_Pne_tmp)
									+ h_Pne_tmp
											* (cos(q[0]) * cos(q[1]) * cos(q[2])
													- cos(q[0]) * sin(q[1])
															* sin(q[2]))))
					+ 20.0 * Pne_tmp) - 380.0 * Pne_tmp * d_Pne_tmp;
	Pne[1] = ((l3
			* (g_Pne_tmp
					* (e_Pne_tmp * f_Pne_tmp * b_Pne_tmp
							+ c_Pne_tmp * f_Pne_tmp * d_Pne_tmp)
					- h_Pne_tmp
							* (f_Pne_tmp * d_Pne_tmp * b_Pne_tmp
									- e_Pne_tmp * c_Pne_tmp * f_Pne_tmp))
			- 380.0
					* (sin(q[0]) * sin(q[1]) * sin(q[2])
							- cos(q[1]) * cos(q[2]) * sin(q[0])))
			+ 20.0 * f_Pne_tmp) - 380.0 * f_Pne_tmp * d_Pne_tmp;
	Pne[2] = ((380.0 * (e_Pne_tmp * b_Pne_tmp + c_Pne_tmp * d_Pne_tmp) + 295.89)
			+ 380.0 * e_Pne_tmp)
			- l3
					* (g_Pne_tmp
							* (cos(q[1]) * cos(q[2]) - d_Pne_tmp * b_Pne_tmp)
							- h_Pne_tmp
									* (cos(q[1]) * sin(q[2])
											+ cos(q[2]) * sin(q[1])));
}


/* Function Definitions */
/*
 * Arguments    : const double q[5]
 *                const double qd[5]
 *                double l3
 *                double twist[6]
 * Return Type  : void
 */
void FVK(float q[4], float qd[4], float l3, float twist[3]) {
	float b_twist_tmp;
	float c_twist_tmp;
	float d_twist_tmp;
	float e_twist_tmp;
	float f_twist_tmp;
	float g_twist_tmp;
	float twist_tmp;
	float twist_tmp_tmp;
	float twist_tmp_tmp_tmp;
	/*  joint vel */
	/* offset */
	/*  l3= 269; */
	twist_tmp = cos(q[0]);
	twist_tmp_tmp_tmp = q[1] + q[2];
	twist_tmp_tmp = twist_tmp_tmp_tmp + q[3];
	b_twist_tmp = cos(twist_tmp_tmp);
	c_twist_tmp = sin(q[0]);
	d_twist_tmp = sin(twist_tmp_tmp);
	e_twist_tmp = 380.0 * sin(twist_tmp_tmp_tmp);
	f_twist_tmp = (e_twist_tmp + 380.0 * cos(q[1]))
			- l3 * cos((q[1] + q[2]) + q[3]);
	twist_tmp_tmp = 380.0 * cos(twist_tmp_tmp_tmp);
	twist_tmp_tmp_tmp = 380.0 * sin(q[1]);
	g_twist_tmp = l3 * qd[3];
	twist[0] = ((g_twist_tmp * b_twist_tmp * twist_tmp
			- qd[2] * twist_tmp * (e_twist_tmp - l3 * b_twist_tmp))
			- qd[0] * c_twist_tmp
					* (((twist_tmp_tmp + 20.0) - twist_tmp_tmp_tmp)
							+ l3 * d_twist_tmp))
			- qd[1] * twist_tmp * f_twist_tmp;
	twist[1] =
			((qd[0] * twist_tmp
					* (((380.0 * cos(q[1] + q[2]) + 20.0) - 380.0 * sin(q[1]))
							+ l3 * sin((q[1] + q[2]) + q[3]))
					- qd[2] * c_twist_tmp
							* (380.0 * sin(q[1] + q[2])
									- l3 * cos((q[1] + q[2]) + q[3])))
					- qd[1] * c_twist_tmp * f_twist_tmp)
					+ l3 * qd[3] * cos((q[1] + q[2]) + q[3]) * c_twist_tmp;
	twist_tmp = l3 * sin((q[1] + q[2]) + q[3]);
	twist[2] = (qd[1] * ((twist_tmp_tmp - twist_tmp_tmp_tmp) + twist_tmp)
			+ qd[2] * (twist_tmp_tmp + twist_tmp)) + g_twist_tmp * d_twist_tmp;
}

void ChessPose(uint8_t Chess_Index, float Chess_Theta, float ChessPosition[2]) {
	/*
	 *
	 */
	uint8_t X = Chess_Index%8;
	uint8_t N = Chess_Index/8;

	float l = 230;
	float L = 400;

	float b_positionx_tmp;
	float c_positionx_tmp;
	float d_positionx_tmp;
	float e_positionx_tmp;
	float positionx_tmp;
	positionx_tmp = cos(Chess_Theta);
	b_positionx_tmp = sin(Chess_Theta);
	c_positionx_tmp = 9.0 * L / 16.0;
	d_positionx_tmp = X * L / 8.0 - c_positionx_tmp;
	e_positionx_tmp = N * L / 8.0;
	ChessPosition[0] = ((b_positionx_tmp * d_positionx_tmp
			+ (e_positionx_tmp - c_positionx_tmp * positionx_tmp)) + l)
			+ L / 2.0;
	ChessPosition[1] = -positionx_tmp * d_positionx_tmp
			+ (e_positionx_tmp - c_positionx_tmp) * b_positionx_tmp;
}

void FindR(float Offset2Center[2], float ChessPosition[2],
		float *ChessRadius) {
	float DeltaX = ChessPosition[0] - Offset2Center[0];
	float DeltaY = ChessPosition[1] - Offset2Center[1];
	*ChessRadius = sqrt((DeltaX * DeltaX) + (DeltaY * DeltaY));
}

void FindXYSpeed(float Chess_Theta, float Chess_Omaga, float ChessRadius, float SpeedXY[2]){
	float SpeedTangent = Chess_Omaga * ChessRadius;
	float Theta = Chess_Theta + (PI/2);
	SpeedXY[0] = SpeedTangent*sin(Theta);
	SpeedXY[1] = SpeedTangent*cos(Theta);
}
