/*
 * Academic License - for use in teaching, academic research, and meeting
 * course requirements at degree granting institutions only.  Not for
 * government, commercial, or other organizational use.
 * File: kalmanFNC.h
 *
 * MATLAB Coder version            : 5.3
 * C/C++ source code generated on  : 06-Feb-2022 21:36:36
 */

#ifndef KALMANFNC_H
#define KALMANFNC_H

/* Include Files */
#include "rtwtypes.h"
#include <stddef.h>
#include <stdlib.h>

#ifdef __cplusplus
extern "C" {
#endif

/* Function Declarations */
extern void kalmanFNC(double R, double Q, double dt, double theta_k, double x1,
                      double x2, double p11, double p12, double p21, double p22,
                      double *xx1, double *xx2, double *pp11, double *pp12,
                      double *pp21, double *pp22);

#ifdef __cplusplus
}
#endif

#endif
/*
 * File trailer for kalmanFNC.h
 *
 * [EOF]
 */
