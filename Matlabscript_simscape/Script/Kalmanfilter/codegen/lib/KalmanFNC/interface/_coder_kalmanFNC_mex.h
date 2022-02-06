/*
 * Academic License - for use in teaching, academic research, and meeting
 * course requirements at degree granting institutions only.  Not for
 * government, commercial, or other organizational use.
 * File: _coder_kalmanFNC_mex.h
 *
 * MATLAB Coder version            : 5.3
 * C/C++ source code generated on  : 06-Feb-2022 21:36:36
 */

#ifndef _CODER_KALMANFNC_MEX_H
#define _CODER_KALMANFNC_MEX_H

/* Include Files */
#include "emlrt.h"
#include "mex.h"
#include "tmwtypes.h"

#ifdef __cplusplus
extern "C" {
#endif

/* Function Declarations */
MEXFUNCTION_LINKAGE void mexFunction(int32_T nlhs, mxArray *plhs[],
                                     int32_T nrhs, const mxArray *prhs[]);

emlrtCTX mexFunctionCreateRootTLS(void);

void unsafe_kalmanFNC_mexFunction(int32_T nlhs, mxArray *plhs[6], int32_T nrhs,
                                  const mxArray *prhs[10]);

#ifdef __cplusplus
}
#endif

#endif
/*
 * File trailer for _coder_kalmanFNC_mex.h
 *
 * [EOF]
 */
