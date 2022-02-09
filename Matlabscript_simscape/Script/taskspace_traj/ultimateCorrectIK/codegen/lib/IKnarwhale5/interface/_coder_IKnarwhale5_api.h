/*
 * Academic License - for use in teaching, academic research, and meeting
 * course requirements at degree granting institutions only.  Not for
 * government, commercial, or other organizational use.
 * File: _coder_IKnarwhale5_api.h
 *
 * MATLAB Coder version            : 5.3
 * C/C++ source code generated on  : 09-Feb-2022 19:42:20
 */

#ifndef _CODER_IKNARWHALE5_API_H
#define _CODER_IKNARWHALE5_API_H

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
void IKnarwhale5(real_T chi[4], real_T gammabar[3], real_T qbar[5]);

void IKnarwhale5_api(const mxArray *const prhs[2], const mxArray **plhs);

void IKnarwhale5_atexit(void);

void IKnarwhale5_initialize(void);

void IKnarwhale5_terminate(void);

void IKnarwhale5_xil_shutdown(void);

void IKnarwhale5_xil_terminate(void);

#ifdef __cplusplus
}
#endif

#endif
/*
 * File trailer for _coder_IKnarwhale5_api.h
 *
 * [EOF]
 */
