/*
 * Academic License - for use in teaching, academic research, and meeting
 * course requirements at degree granting institutions only.  Not for
 * government, commercial, or other organizational use.
 * File: _coder_kalmanFNC_api.h
 *
 * MATLAB Coder version            : 5.3
 * C/C++ source code generated on  : 06-Feb-2022 21:36:36
 */

#ifndef _CODER_KALMANFNC_API_H
#define _CODER_KALMANFNC_API_H

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
void kalmanFNC(real_T R, real_T Q, real_T dt, real_T theta_k, real_T x1,
               real_T x2, real_T p11, real_T p12, real_T p21, real_T p22,
               real_T *xx1, real_T *xx2, real_T *pp11, real_T *pp12,
               real_T *pp21, real_T *pp22);

void kalmanFNC_api(const mxArray *const prhs[10], int32_T nlhs,
                   const mxArray *plhs[6]);

void kalmanFNC_atexit(void);

void kalmanFNC_initialize(void);

void kalmanFNC_terminate(void);

void kalmanFNC_xil_shutdown(void);

void kalmanFNC_xil_terminate(void);

#ifdef __cplusplus
}
#endif

#endif
/*
 * File trailer for _coder_kalmanFNC_api.h
 *
 * [EOF]
 */
