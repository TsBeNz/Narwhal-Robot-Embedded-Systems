/*
 * Academic License - for use in teaching, academic research, and meeting
 * course requirements at degree granting institutions only.  Not for
 * government, commercial, or other organizational use.
 * File: planningTraj_emxutil.h
 *
 * MATLAB Coder version            : 5.3
 * C/C++ source code generated on  : 18-Apr-2022 13:57:29
 */

#ifndef PLANNINGTRAJ_EMXUTIL_H
#define PLANNINGTRAJ_EMXUTIL_H

/* Include Files */
#include "planningTraj_types.h"
#include "rtwtypes.h"
#include <stddef.h>
#include <stdlib.h>

#ifdef __cplusplus
extern "C" {
#endif

/* Function Declarations */
extern void emxEnsureCapacity_real_T(emxArray_real_T *emxArray, int oldNumel);

extern void emxFree_real_T(emxArray_real_T **pEmxArray);

extern void emxInit_real_T(emxArray_real_T **pEmxArray, int numDimensions);

#ifdef __cplusplus
}
#endif

#endif
/*
 * File trailer for planningTraj_emxutil.h
 *
 * [EOF]
 */
