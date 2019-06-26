/*
 * Academic License - for use in teaching, academic research, and meeting
 * course requirements at degree granting institutions only.  Not for
 * government, commercial, or other organizational use.
 *
 * testMEX.c
 *
 * Code generation for function 'testMEX'
 *
 */

/* Include files */
#include "rt_nonfinite.h"
#include "testMEX.h"
#include "testMEX_data.h"

/* Variable Definitions */
static emlrtBCInfo emlrtBCI = { 0,     /* iFirst */
  1,                                   /* iLast */
  4,                                   /* lineNo */
  9,                                   /* colNo */
  "c",                                 /* aName */
  "testMEX",                           /* fName */
  "C:\\Users\\jessl\\Documents\\GitHub\\MM\\testMEX.m",/* pName */
  3                                    /* checkKind */
};

/* Function Definitions */
void testMEX(const emlrtStack *sp, real_T a, real_T b, real_T c[2])
{
  int32_T i;
  for (i = 0; i < 2; i++) {
    c[i] = 1.0 + 999.0 * (real_T)i;
  }

  i = 0;
  while (i < 1000) {
    if (!(i <= 1)) {
      emlrtDynamicBoundsCheckR2012b(i, 0, 1, &emlrtBCI, sp);
    }

    c[i] = a + b;
    i++;
    if (*emlrtBreakCheckR2012bFlagVar != 0) {
      emlrtBreakCheckR2012b(sp);
    }
  }
}

/* End of code generation (testMEX.c) */
