/*
 * Academic License - for use in teaching, academic research, and meeting
 * course requirements at degree granting institutions only.  Not for
 * government, commercial, or other organizational use.
 *
 * _coder_testMEX_api.c
 *
 * Code generation for function '_coder_testMEX_api'
 *
 */

/* Include files */
#include "rt_nonfinite.h"
#include "testMEX.h"
#include "_coder_testMEX_api.h"
#include "testMEX_data.h"

/* Function Declarations */
static real_T b_emlrt_marshallIn(const emlrtStack *sp, const mxArray *u, const
  emlrtMsgIdentifier *parentId);
static real_T c_emlrt_marshallIn(const emlrtStack *sp, const mxArray *src, const
  emlrtMsgIdentifier *msgId);
static real_T emlrt_marshallIn(const emlrtStack *sp, const mxArray *a, const
  char_T *identifier);
static const mxArray *emlrt_marshallOut(const real_T u[2]);

/* Function Definitions */
static real_T b_emlrt_marshallIn(const emlrtStack *sp, const mxArray *u, const
  emlrtMsgIdentifier *parentId)
{
  real_T y;
  y = c_emlrt_marshallIn(sp, emlrtAlias(u), parentId);
  emlrtDestroyArray(&u);
  return y;
}

static real_T c_emlrt_marshallIn(const emlrtStack *sp, const mxArray *src, const
  emlrtMsgIdentifier *msgId)
{
  real_T ret;
  static const int32_T dims = 0;
  emlrtCheckBuiltInR2012b(sp, msgId, src, "double", false, 0U, &dims);
  ret = *(real_T *)emlrtMxGetData(src);
  emlrtDestroyArray(&src);
  return ret;
}

static real_T emlrt_marshallIn(const emlrtStack *sp, const mxArray *a, const
  char_T *identifier)
{
  real_T y;
  emlrtMsgIdentifier thisId;
  thisId.fIdentifier = (const char *)identifier;
  thisId.fParent = NULL;
  thisId.bParentIsCell = false;
  y = b_emlrt_marshallIn(sp, emlrtAlias(a), &thisId);
  emlrtDestroyArray(&a);
  return y;
}

static const mxArray *emlrt_marshallOut(const real_T u[2])
{
  const mxArray *y;
  int32_T i0;
  int32_T iv1[2];
  real_T b_u;
  const mxArray *b_y;
  const mxArray *m1;
  y = NULL;
  for (i0 = 0; i0 < 2; i0++) {
    iv1[i0] = 1 + i0;
  }

  emlrtAssign(&y, emlrtCreateCellArrayR2014a(2, iv1));
  for (i0 = 0; i0 < 2; i0++) {
    b_u = u[i0];
    b_y = NULL;
    m1 = emlrtCreateDoubleScalar(b_u);
    emlrtAssign(&b_y, m1);
    emlrtSetCell(y, i0, b_y);
  }

  return y;
}

void testMEX_api(const mxArray * const prhs[2], const mxArray *plhs[1])
{
  real_T a;
  real_T b;
  real_T c[2];
  emlrtStack st = { NULL,              /* site */
    NULL,                              /* tls */
    NULL                               /* prev */
  };

  st.tls = emlrtRootTLSGlobal;

  /* Marshall function inputs */
  a = emlrt_marshallIn(&st, emlrtAliasP(prhs[0]), "a");
  b = emlrt_marshallIn(&st, emlrtAliasP(prhs[1]), "b");

  /* Invoke the target function */
  testMEX(&st, a, b, c);

  /* Marshall function outputs */
  plhs[0] = emlrt_marshallOut(c);
}

/* End of code generation (_coder_testMEX_api.c) */
