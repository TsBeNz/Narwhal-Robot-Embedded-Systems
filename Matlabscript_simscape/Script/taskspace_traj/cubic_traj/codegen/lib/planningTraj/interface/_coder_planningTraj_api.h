/*
 * Academic License - for use in teaching, academic research, and meeting
 * course requirements at degree granting institutions only.  Not for
 * government, commercial, or other organizational use.
 * File: _coder_planningTraj_api.h
 *
 * MATLAB Coder version            : 5.3
 * C/C++ source code generated on  : 18-Apr-2022 13:57:29
 */

#ifndef _CODER_PLANNINGTRAJ_API_H
#define _CODER_PLANNINGTRAJ_API_H

/* Include Files */
#include "emlrt.h"
#include "tmwtypes.h"
#include <string.h>

/* Type Definitions */
#ifndef struct_emxArray_real_T
#define struct_emxArray_real_T
struct emxArray_real_T {
  real_T *data;
  int32_T *size;
  int32_T allocatedSize;
  int32_T numDimensions;
  boolean_T canFreeData;
};
#endif /* struct_emxArray_real_T */
#ifndef typedef_emxArray_real_T
#define typedef_emxArray_real_T
typedef struct emxArray_real_T emxArray_real_T;
#endif /* typedef_emxArray_real_T */

/* Variable Declarations */
extern emlrtCTX emlrtRootTLSGlobal;
extern emlrtContext emlrtContextGlobal;

#ifdef __cplusplus
extern "C" {
#endif

/* Function Declarations */
void planningTraj(real_T q0, real_T q1, real_T v0, real_T v1, real_T tf,
                  emxArray_real_T *q, emxArray_real_T *v);

void planningTraj_api(const mxArray *const prhs[5], int32_T nlhs,
                      const mxArray *plhs[2]);

void planningTraj_atexit(void);

void planningTraj_initialize(void);

void planningTraj_terminate(void);

void planningTraj_xil_shutdown(void);

void planningTraj_xil_terminate(void);

#ifdef __cplusplus
}
#endif

#endif
/*
 * File trailer for _coder_planningTraj_api.h
 *
 * [EOF]
 */
