/*
 * Academic License - for use in teaching, academic research, and meeting
 * course requirements at degree granting institutions only.  Not for
 * government, commercial, or other organizational use.
 * File: FPK.c
 *
 * MATLAB Coder version            : 5.3
 * C/C++ source code generated on  : 21-Feb-2022 17:01:06
 */

/* Include Files */
#include "FPK.h"
#include <math.h>

/* Function Definitions */
/*
 * Arguments    : const double q[5]
 *                double l3
 *                double Pne[3]
 * Return Type  : void
 */
void FPK(const double q[5], double l3, double Pne[3])
{
  double Pne_tmp;
  double b_Pne_tmp;
  double c_Pne_tmp;
  double d_Pne_tmp;
  double e_Pne_tmp;
  double f_Pne_tmp;
  double g_Pne_tmp;
  double h_Pne_tmp;
  /* offset */
  Pne_tmp = cos(q[0]);
  b_Pne_tmp = sin(q[2]);
  c_Pne_tmp = cos(q[2]);
  d_Pne_tmp = sin(q[1]);
  e_Pne_tmp = cos(q[1]);
  f_Pne_tmp = sin(q[0]);
  g_Pne_tmp = cos(q[3]);
  h_Pne_tmp = sin(q[3]);
  Pne[0] = ((380.0 * (Pne_tmp * e_Pne_tmp * c_Pne_tmp -
                      Pne_tmp * d_Pne_tmp * b_Pne_tmp) +
             l3 * (g_Pne_tmp * (cos(q[0]) * cos(q[1]) * b_Pne_tmp +
                                Pne_tmp * c_Pne_tmp * d_Pne_tmp) +
                   h_Pne_tmp * (cos(q[0]) * cos(q[1]) * cos(q[2]) -
                                cos(q[0]) * sin(q[1]) * sin(q[2])))) +
            20.0 * Pne_tmp) -
           380.0 * Pne_tmp * d_Pne_tmp;
  Pne[1] = ((l3 * (g_Pne_tmp * (e_Pne_tmp * f_Pne_tmp * b_Pne_tmp +
                                c_Pne_tmp * f_Pne_tmp * d_Pne_tmp) -
                   h_Pne_tmp * (f_Pne_tmp * d_Pne_tmp * b_Pne_tmp -
                                e_Pne_tmp * c_Pne_tmp * f_Pne_tmp)) -
             380.0 * (sin(q[0]) * sin(q[1]) * sin(q[2]) -
                      cos(q[1]) * cos(q[2]) * sin(q[0]))) +
            20.0 * f_Pne_tmp) -
           380.0 * f_Pne_tmp * d_Pne_tmp;
  Pne[2] = ((380.0 * (e_Pne_tmp * b_Pne_tmp + c_Pne_tmp * d_Pne_tmp) + 295.89) +
            380.0 * e_Pne_tmp) -
           l3 * (g_Pne_tmp * (cos(q[1]) * cos(q[2]) - d_Pne_tmp * b_Pne_tmp) -
                 h_Pne_tmp * (cos(q[1]) * sin(q[2]) + cos(q[2]) * sin(q[1])));
}

/*
 * File trailer for FPK.c
 *
 * [EOF]
 */
