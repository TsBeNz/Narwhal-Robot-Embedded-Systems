/*
 * Academic License - for use in teaching, academic research, and meeting
 * course requirements at degree granting institutions only.  Not for
 * government, commercial, or other organizational use.
 * File: _coder_kalmanFNC_api.c
 *
 * MATLAB Coder version            : 5.3
 * C/C++ source code generated on  : 06-Feb-2022 21:36:36
 */

/* Include Files */
#include "_coder_kalmanFNC_api.h"
#include "_coder_kalmanFNC_mex.h"

/* Variable Definitions */
emlrtCTX emlrtRootTLSGlobal = NULL;

emlrtContext emlrtContextGlobal = {
    true,                                                 /* bFirstTime */
    false,                                                /* bInitialized */
    131611U,                                              /* fVersionInfo */
    NULL,                                                 /* fErrorFunction */
    "kalmanFNC",                                          /* fFunctionName */
    NULL,                                                 /* fRTCallStack */
    false,                                                /* bDebugMode */
    {2045744189U, 2170104910U, 2743257031U, 4284093946U}, /* fSigWrd */
    NULL                                                  /* fSigMem */
};

/* Function Declarations */
static real_T b_emlrt_marshallIn(const emlrtStack *sp, const mxArray *u,
                                 const emlrtMsgIdentifier *parentId);

static real_T c_emlrt_marshallIn(const emlrtStack *sp, const mxArray *src,
                                 const emlrtMsgIdentifier *msgId);

static real_T emlrt_marshallIn(const emlrtStack *sp, const mxArray *R,
                               const char_T *identifier);

static const mxArray *emlrt_marshallOut(const real_T u);

/* Function Definitions */
/*
 * Arguments    : const emlrtStack *sp
 *                const mxArray *u
 *                const emlrtMsgIdentifier *parentId
 * Return Type  : real_T
 */
static real_T b_emlrt_marshallIn(const emlrtStack *sp, const mxArray *u,
                                 const emlrtMsgIdentifier *parentId)
{
  real_T y;
  y = c_emlrt_marshallIn(sp, emlrtAlias(u), parentId);
  emlrtDestroyArray(&u);
  return y;
}

/*
 * Arguments    : const emlrtStack *sp
 *                const mxArray *src
 *                const emlrtMsgIdentifier *msgId
 * Return Type  : real_T
 */
static real_T c_emlrt_marshallIn(const emlrtStack *sp, const mxArray *src,
                                 const emlrtMsgIdentifier *msgId)
{
  static const int32_T dims = 0;
  real_T ret;
  emlrtCheckBuiltInR2012b((emlrtCTX)sp, msgId, src, (const char_T *)"double",
                          false, 0U, (void *)&dims);
  ret = *(real_T *)emlrtMxGetData(src);
  emlrtDestroyArray(&src);
  return ret;
}

/*
 * Arguments    : const emlrtStack *sp
 *                const mxArray *R
 *                const char_T *identifier
 * Return Type  : real_T
 */
static real_T emlrt_marshallIn(const emlrtStack *sp, const mxArray *R,
                               const char_T *identifier)
{
  emlrtMsgIdentifier thisId;
  real_T y;
  thisId.fIdentifier = (const char_T *)identifier;
  thisId.fParent = NULL;
  thisId.bParentIsCell = false;
  y = b_emlrt_marshallIn(sp, emlrtAlias(R), &thisId);
  emlrtDestroyArray(&R);
  return y;
}

/*
 * Arguments    : const real_T u
 * Return Type  : const mxArray *
 */
static const mxArray *emlrt_marshallOut(const real_T u)
{
  const mxArray *m;
  const mxArray *y;
  y = NULL;
  m = emlrtCreateDoubleScalar(u);
  emlrtAssign(&y, m);
  return y;
}

/*
 * Arguments    : const mxArray * const prhs[10]
 *                int32_T nlhs
 *                const mxArray *plhs[6]
 * Return Type  : void
 */
void kalmanFNC_api(const mxArray *const prhs[10], int32_T nlhs,
                   const mxArray *plhs[6])
{
  emlrtStack st = {
      NULL, /* site */
      NULL, /* tls */
      NULL  /* prev */
  };
  real_T Q;
  real_T R;
  real_T dt;
  real_T p11;
  real_T p12;
  real_T p21;
  real_T p22;
  real_T pp11;
  real_T pp12;
  real_T pp21;
  real_T pp22;
  real_T theta_k;
  real_T x1;
  real_T x2;
  real_T xx1;
  real_T xx2;
  st.tls = emlrtRootTLSGlobal;
  /* Marshall function inputs */
  R = emlrt_marshallIn(&st, emlrtAliasP(prhs[0]), "R");
  Q = emlrt_marshallIn(&st, emlrtAliasP(prhs[1]), "Q");
  dt = emlrt_marshallIn(&st, emlrtAliasP(prhs[2]), "dt");
  theta_k = emlrt_marshallIn(&st, emlrtAliasP(prhs[3]), "theta_k");
  x1 = emlrt_marshallIn(&st, emlrtAliasP(prhs[4]), "x1");
  x2 = emlrt_marshallIn(&st, emlrtAliasP(prhs[5]), "x2");
  p11 = emlrt_marshallIn(&st, emlrtAliasP(prhs[6]), "p11");
  p12 = emlrt_marshallIn(&st, emlrtAliasP(prhs[7]), "p12");
  p21 = emlrt_marshallIn(&st, emlrtAliasP(prhs[8]), "p21");
  p22 = emlrt_marshallIn(&st, emlrtAliasP(prhs[9]), "p22");
  /* Invoke the target function */
  kalmanFNC(R, Q, dt, theta_k, x1, x2, p11, p12, p21, p22, &xx1, &xx2, &pp11,
            &pp12, &pp21, &pp22);
  /* Marshall function outputs */
  plhs[0] = emlrt_marshallOut(xx1);
  if (nlhs > 1) {
    plhs[1] = emlrt_marshallOut(xx2);
  }
  if (nlhs > 2) {
    plhs[2] = emlrt_marshallOut(pp11);
  }
  if (nlhs > 3) {
    plhs[3] = emlrt_marshallOut(pp12);
  }
  if (nlhs > 4) {
    plhs[4] = emlrt_marshallOut(pp21);
  }
  if (nlhs > 5) {
    plhs[5] = emlrt_marshallOut(pp22);
  }
}

/*
 * Arguments    : void
 * Return Type  : void
 */
void kalmanFNC_atexit(void)
{
  emlrtStack st = {
      NULL, /* site */
      NULL, /* tls */
      NULL  /* prev */
  };
  mexFunctionCreateRootTLS();
  st.tls = emlrtRootTLSGlobal;
  emlrtEnterRtStackR2012b(&st);
  emlrtLeaveRtStackR2012b(&st);
  emlrtDestroyRootTLS(&emlrtRootTLSGlobal);
  kalmanFNC_xil_terminate();
  kalmanFNC_xil_shutdown();
  emlrtExitTimeCleanup(&emlrtContextGlobal);
}

/*
 * Arguments    : void
 * Return Type  : void
 */
void kalmanFNC_initialize(void)
{
  emlrtStack st = {
      NULL, /* site */
      NULL, /* tls */
      NULL  /* prev */
  };
  mexFunctionCreateRootTLS();
  st.tls = emlrtRootTLSGlobal;
  emlrtClearAllocCountR2012b(&st, false, 0U, NULL);
  emlrtEnterRtStackR2012b(&st);
  emlrtFirstTimeR2012b(emlrtRootTLSGlobal);
}

/*
 * Arguments    : void
 * Return Type  : void
 */
void kalmanFNC_terminate(void)
{
  emlrtStack st = {
      NULL, /* site */
      NULL, /* tls */
      NULL  /* prev */
  };
  st.tls = emlrtRootTLSGlobal;
  emlrtLeaveRtStackR2012b(&st);
  emlrtDestroyRootTLS(&emlrtRootTLSGlobal);
}

/*
 * File trailer for _coder_kalmanFNC_api.c
 *
 * [EOF]
 */
