/*
 * Academic License - for use in teaching, academic research, and meeting
 * course requirements at degree granting institutions only.  Not for
 * government, commercial, or other organizational use.
 * File: _coder_IVK_api.h
 *
 * MATLAB Coder version            : 5.3
 * C/C++ source code generated on  : 28-Feb-2022 21:25:14
 */

#ifndef _CODER_IVK_API_H
#define _CODER_IVK_API_H

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
void IVK(real_T q[4], real_T chi_dot[3], real_T qv[4]);

void IVK_api(const mxArray *const prhs[2], const mxArray **plhs);

void IVK_atexit(void);

void IVK_initialize(void);

void IVK_terminate(void);

void IVK_xil_shutdown(void);

void IVK_xil_terminate(void);

#ifdef __cplusplus
}
#endif

#endif
/*
 * File trailer for _coder_IVK_api.h
 *
 * [EOF]
 */
