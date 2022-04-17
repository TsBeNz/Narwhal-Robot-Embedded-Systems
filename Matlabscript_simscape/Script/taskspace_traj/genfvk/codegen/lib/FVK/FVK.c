/*
 * Academic License - for use in teaching, academic research, and meeting
 * course requirements at degree granting institutions only.  Not for
 * government, commercial, or other organizational use.
 * File: FVK.c
 *
 * MATLAB Coder version            : 5.3
 * C/C++ source code generated on  : 13-Apr-2022 17:16:59
 */

/* Include Files */
#include "FVK.h"
#include <math.h>

/* Function Definitions */
/*
 * Arguments    : const double q[5]
 *                const double qd[5]
 *                double l3
 *                double twist[3]
 * Return Type  : void
 */
void FVK(const double q[5], const double qd[5], double l3, double twist[3])
{
  double b_twist_tmp;
  double c_twist_tmp;
  double d_twist_tmp;
  double e_twist_tmp;
  double f_twist_tmp;
  double g_twist_tmp;
  double twist_tmp;
  double twist_tmp_tmp;
  double twist_tmp_tmp_tmp;
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
  f_twist_tmp =
      (e_twist_tmp + 380.0 * cos(q[1])) - l3 * cos((q[1] + q[2]) + q[3]);
  twist_tmp_tmp = 380.0 * cos(twist_tmp_tmp_tmp);
  twist_tmp_tmp_tmp = 380.0 * sin(q[1]);
  g_twist_tmp = l3 * qd[3];
  twist[0] =
      ((g_twist_tmp * b_twist_tmp * twist_tmp -
        qd[2] * twist_tmp * (e_twist_tmp - l3 * b_twist_tmp)) -
       qd[0] * c_twist_tmp *
           (((twist_tmp_tmp + 20.0) - twist_tmp_tmp_tmp) + l3 * d_twist_tmp)) -
      qd[1] * twist_tmp * f_twist_tmp;
  twist[1] =
      ((qd[0] * twist_tmp *
            (((380.0 * cos(q[1] + q[2]) + 20.0) - 380.0 * sin(q[1])) +
             l3 * sin((q[1] + q[2]) + q[3])) -
        qd[2] * c_twist_tmp *
            (380.0 * sin(q[1] + q[2]) - l3 * cos((q[1] + q[2]) + q[3]))) -
       qd[1] * c_twist_tmp * f_twist_tmp) +
      l3 * qd[3] * cos((q[1] + q[2]) + q[3]) * c_twist_tmp;
  twist_tmp = l3 * sin((q[1] + q[2]) + q[3]);
  twist[2] = (qd[1] * ((twist_tmp_tmp - twist_tmp_tmp_tmp) + twist_tmp) +
              qd[2] * (twist_tmp_tmp + twist_tmp)) +
             g_twist_tmp * d_twist_tmp;
}

/*
 * File trailer for FVK.c
 *
 * [EOF]
 */
