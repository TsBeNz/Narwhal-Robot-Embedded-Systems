/*
 * Academic License - for use in teaching, academic research, and meeting
 * course requirements at degree granting institutions only.  Not for
 * government, commercial, or other organizational use.
 * File: IKnarwhale5.c
 *
 * MATLAB Coder version            : 5.3
 * C/C++ source code generated on  : 21-Feb-2022 17:08:41
 */

/* Include Files */
#include "IKnarwhale5.h"
#include "rt_nonfinite.h"
#include "rt_defines.h"
#include "rt_nonfinite.h"
#include <math.h>

/* Function Declarations */
static double rt_atan2d_snf(double u0, double u1);

/* Function Definitions */
/*
 * Arguments    : double u0
 *                double u1
 * Return Type  : double
 */
static double rt_atan2d_snf(double u0, double u1)
{
  double y;
  int b_u0;
  int b_u1;
  if (rtIsNaN(u0) || rtIsNaN(u1)) {
    y = rtNaN;
  } else if (rtIsInf(u0) && rtIsInf(u1)) {
    if (u0 > 0.0) {
      b_u0 = 1;
    } else {
      b_u0 = -1;
    }
    if (u1 > 0.0) {
      b_u1 = 1;
    } else {
      b_u1 = -1;
    }
    y = atan2(b_u0, b_u1);
  } else if (u1 == 0.0) {
    if (u0 > 0.0) {
      y = RT_PI / 2.0;
    } else if (u0 < 0.0) {
      y = -(RT_PI / 2.0);
    } else {
      y = 0.0;
    }
  } else {
    y = atan2(u0, u1);
  }
  return y;
}

/*
 * Arguments    : const double chi[4]
 *                const double gammabar[3]
 *                double l3
 *                double qbar[5]
 * Return Type  : void
 */
void IKnarwhale5(const double chi[4], const double gammabar[3], double l3,
                 double qbar[5])
{
  double c2;
  double q2;
  double q3;
  double s2;
  double x24;
  double z24;
  /*  RRIK */
  z24 = (chi[2] + l3) - 295.89;
  x24 = gammabar[1] * sqrt(chi[0] * chi[0] + chi[1] * chi[1]) - 20.0;
  c2 = x24 * x24 + z24 * z24;
  s2 = sqrt(c2);
  if ((s2 <= 760.0) && (s2 >= 0.0)) {
    c2 = ((c2 - 144400.0) - 144400.0) / 288800.0;
    s2 = gammabar[2] * sqrt(1.0 - c2 * c2);
    q2 = (rt_atan2d_snf(z24, x24) -
          rt_atan2d_snf(380.0 * s2, 380.0 * c2 + 380.0)) -
         1.5707963267948966;
    q3 = rt_atan2d_snf(s2, c2) + 1.5707963267948966;
  } else {
    /*  สั่งเกินระยะแขน เกิดท่าประหลาด */
  }
  qbar[0] = rt_atan2d_snf(gammabar[0] * chi[1], gammabar[0] * chi[0]);
  qbar[1] = q2;
  qbar[2] = q3;
  qbar[3] = -q2 - q3;
  qbar[4] = chi[3];
}

/*
 * File trailer for IKnarwhale5.c
 *
 * [EOF]
 */
