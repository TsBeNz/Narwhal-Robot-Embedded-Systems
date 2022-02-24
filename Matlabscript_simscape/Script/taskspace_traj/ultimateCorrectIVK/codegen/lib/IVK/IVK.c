/*
 * Academic License - for use in teaching, academic research, and meeting
 * course requirements at degree granting institutions only.  Not for
 * government, commercial, or other organizational use.
 * File: IVK.c
 *
 * MATLAB Coder version            : 5.3
 * C/C++ source code generated on  : 21-Feb-2022 17:10:32
 */

/* Include Files */
#include "IVK.h"
#include <math.h>

/* Function Definitions */
/*
 * q,qv5,chi_dot  % q,taskspace
 *
 * Arguments    : const double q[5]
 *                double qv5
 *                const double chi_dot[3]
 *                double qv[5]
 * Return Type  : void
 */
void IVK(const double q[5], double qv5, const double chi_dot[3], double qv[5])
{
  double Jv4[9];
  double Jv4_tmp;
  double Jv4_tmp_tmp;
  double b_Jv4_tmp;
  double c_Jv4_tmp;
  double qvbar_idx_1;
  double qvbar_idx_2;
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
  qv[4] = qv5;
}

/*
 * File trailer for IVK.c
 *
 * [EOF]
 */
