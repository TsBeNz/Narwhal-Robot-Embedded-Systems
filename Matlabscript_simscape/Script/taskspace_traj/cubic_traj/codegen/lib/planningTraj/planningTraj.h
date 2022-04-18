/*
 * Academic License - for use in teaching, academic research, and meeting
 * course requirements at degree granting institutions only.  Not for
 * government, commercial, or other organizational use.
 * File: planningTraj.h
 *
 * MATLAB Coder version            : 5.3
 * C/C++ source code generated on  : 18-Apr-2022 13:57:29
 */

#ifndef PLANNINGTRAJ_H
#define PLANNINGTRAJ_H

/* Include Files */
#include "planningTraj_types.h"
#include "rtwtypes.h"
#include <stddef.h>
#include <stdlib.h>

#ifdef __cplusplus
extern "C" {
#endif

/* Function Declarations */
extern void planningTraj(double q0, double q1, double v0, double v1, double tf,
                         emxArray_real_T *q, emxArray_real_T *v);

#ifdef __cplusplus
}
#endif

#endif
/*
 * File trailer for planningTraj.h
 *
 * [EOF]
 */
