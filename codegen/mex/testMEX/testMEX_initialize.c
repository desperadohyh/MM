/*
 * Academic License - for use in teaching, academic research, and meeting
 * course requirements at degree granting institutions only.  Not for
 * government, commercial, or other organizational use.
 *
 * testMEX_initialize.c
 *
 * Code generation for function 'testMEX_initialize'
 *
 */

/* Include files */
#include "rt_nonfinite.h"
#include "testMEX.h"
#include "testMEX_initialize.h"
#include "_coder_testMEX_mex.h"
#include "testMEX_data.h"

/* Function Definitions */
void testMEX_initialize(void)
{
  emlrtStack st = { NULL,              /* site */
    NULL,                              /* tls */
    NULL                               /* prev */
  };

  mexFunctionCreateRootTLS();
  emlrtBreakCheckR2012bFlagVar = emlrtGetBreakCheckFlagAddressR2012b();
  st.tls = emlrtRootTLSGlobal;
  emlrtClearAllocCountR2012b(&st, false, 0U, 0);
  emlrtEnterRtStackR2012b(&st);
  emlrtFirstTimeR2012b(emlrtRootTLSGlobal);
}

/* End of code generation (testMEX_initialize.c) */
