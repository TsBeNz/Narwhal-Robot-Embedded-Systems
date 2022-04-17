/*
 * Academic License - for use in teaching, academic research, and meeting
 * course requirements at degree granting institutions only.  Not for
 * government, commercial, or other organizational use.
 * File: _coder_FVK_api.h
 *
 * MATLAB Coder version            : 5.3
 * C/C++ source code generated on  : 13-Apr-2022 17:16:59
 */

#ifndef _CODER_FVK_API_H
#define _CODER_FVK_API_H

/* Include Files */
#include "emlrt.h"
#include "tmwtypes.h"
#include <string.h>

/* Variable Declarations */
extern emlrtCTX emlrtRootTLSGlobal;
extern emlrtContext emlrtContextGlobal;

#ifdef __cplusplus
extern "C" {
#endif

/* Function Declarations */
void FVK(real_T q[5], real_T qd[5], real_T l3, real_T twist[3]);

void FVK_api(const mxArray *const prhs[3], const mxArray **plhs);

void FVK_atexit(void);

void FVK_initialize(void);

void FVK_terminate(void);

void FVK_xil_shutdown(void);

void FVK_xil_terminate(void);

#ifdef __cplusplus
}
#endif

#endif
/*
 * File trailer for _coder_FVK_api.h
 *
 * [EOF]
 */
