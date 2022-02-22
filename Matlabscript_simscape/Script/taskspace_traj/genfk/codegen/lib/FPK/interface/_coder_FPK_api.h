/*
 * Academic License - for use in teaching, academic research, and meeting
 * course requirements at degree granting institutions only.  Not for
 * government, commercial, or other organizational use.
 * File: _coder_FPK_api.h
 *
 * MATLAB Coder version            : 5.3
 * C/C++ source code generated on  : 21-Feb-2022 17:01:06
 */

#ifndef _CODER_FPK_API_H
#define _CODER_FPK_API_H

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
void FPK(real_T q[5], real_T l3, real_T Pne[3]);

void FPK_api(const mxArray *const prhs[2], const mxArray **plhs);

void FPK_atexit(void);

void FPK_initialize(void);

void FPK_terminate(void);

void FPK_xil_shutdown(void);

void FPK_xil_terminate(void);

#ifdef __cplusplus
}
#endif

#endif
/*
 * File trailer for _coder_FPK_api.h
 *
 * [EOF]
 */
