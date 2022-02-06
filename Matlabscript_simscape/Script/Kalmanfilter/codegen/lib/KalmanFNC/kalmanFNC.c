/*
 * Academic License - for use in teaching, academic research, and meeting
 * course requirements at degree granting institutions only.  Not for
 * government, commercial, or other organizational use.
 * File: kalmanFNC.c
 *
 * MATLAB Coder version            : 5.3
 * C/C++ source code generated on  : 06-Feb-2022 21:36:36
 */

/* Include Files */
#include "kalmanFNC.h"
#include "rt_nonfinite.h"
#include "rt_nonfinite.h"
#include <math.h>

/* Function Declarations */
static double rt_powd_snf(double u0, double u1);

/* Function Definitions */
/*
 * Arguments    : double u0
 *                double u1
 * Return Type  : double
 */
static double rt_powd_snf(double u0, double u1)
{
  double d;
  double d1;
  double y;
  if (rtIsNaN(u0) || rtIsNaN(u1)) {
    y = rtNaN;
  } else {
    d = fabs(u0);
    d1 = fabs(u1);
    if (rtIsInf(u1)) {
      if (d == 1.0) {
        y = 1.0;
      } else if (d > 1.0) {
        if (u1 > 0.0) {
          y = rtInf;
        } else {
          y = 0.0;
        }
      } else if (u1 > 0.0) {
        y = 0.0;
      } else {
        y = rtInf;
      }
    } else if (d1 == 0.0) {
      y = 1.0;
    } else if (d1 == 1.0) {
      if (u1 > 0.0) {
        y = u0;
      } else {
        y = 1.0 / u0;
      }
    } else if (u1 == 2.0) {
      y = u0 * u0;
    } else if ((u1 == 0.5) && (u0 >= 0.0)) {
      y = sqrt(u0);
    } else if ((u0 < 0.0) && (u1 > floor(u1))) {
      y = rtNaN;
    } else {
      y = pow(u0, u1);
    }
  }
  return y;
}

/*
 * Arguments    : double R
 *                double Q
 *                double dt
 *                double theta_k
 *                double x1
 *                double x2
 *                double p11
 *                double p12
 *                double p21
 *                double p22
 *                double *xx1
 *                double *xx2
 *                double *pp11
 *                double *pp12
 *                double *pp21
 *                double *pp22
 * Return Type  : void
 */
void kalmanFNC(double R, double Q, double dt, double theta_k, double x1,
               double x2, double p11, double p12, double p21, double p22,
               double *xx1, double *xx2, double *pp11, double *pp12,
               double *pp21, double *pp22)
{
  double b_xx1_tmp;
  double b_xx2_tmp;
  double c_xx1_tmp;
  double c_xx2_tmp;
  double d_xx1_tmp;
  double d_xx2_tmp;
  double e_xx1_tmp;
  double xx1_tmp;
  double xx1_tmp_tmp;
  double xx2_tmp;
  xx1_tmp = 4.0 * dt * p12;
  b_xx1_tmp = 4.0 * dt * p21;
  c_xx1_tmp = Q * rt_powd_snf(dt, 4.0);
  xx1_tmp_tmp = dt * dt;
  d_xx1_tmp = 4.0 * xx1_tmp_tmp * p22;
  e_xx1_tmp =
      ((((4.0 * R + 4.0 * p11) + xx1_tmp) + b_xx1_tmp) + c_xx1_tmp) + d_xx1_tmp;
  *xx1 = ((((((4.0 * R * x1 + 4.0 * p11 * theta_k) + d_xx1_tmp * theta_k) +
             4.0 * R * dt * x2) +
            xx1_tmp * theta_k) +
           b_xx1_tmp * theta_k) +
          c_xx1_tmp * theta_k) /
         e_xx1_tmp;
  xx2_tmp = p22 * dt;
  b_xx2_tmp = Q * rt_powd_snf(dt, 3.0);
  c_xx2_tmp = b_xx2_tmp / 2.0 + xx2_tmp;
  d_xx2_tmp = c_xx2_tmp + p21;
  xx2_tmp = (((R + p11) + dt * p21) + c_xx1_tmp / 4.0) + dt * (p12 + xx2_tmp);
  *xx2 = x2 - d_xx2_tmp * ((x1 - theta_k) + dt * x2) / xx2_tmp;
  *pp11 = R * ((((4.0 * p11 + xx1_tmp) + b_xx1_tmp) + c_xx1_tmp) + d_xx1_tmp) /
          e_xx1_tmp;
  xx1_tmp = b_xx2_tmp + 2.0 * p22 * dt;
  *pp12 = 2.0 * R * (xx1_tmp + 2.0 * p12) / e_xx1_tmp;
  *pp21 = 2.0 * R * (xx1_tmp + 2.0 * p21) / e_xx1_tmp;
  *pp22 = (p22 + Q * xx1_tmp_tmp) - (c_xx2_tmp + p12) * d_xx2_tmp / xx2_tmp;
}

/*
 * File trailer for kalmanFNC.c
 *
 * [EOF]
 */
