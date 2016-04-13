/*
 * Academic License - for use in teaching, academic research, and meeting
 * course requirements at degree granting institutions only.  Not for
 * government, commercial, or other organizational use.
 * File: _coder_cam2world_api.h
 *
 * MATLAB Coder version            : 2.8.1
 * C/C++ source code generated on  : 11-Mar-2016 16:57:34
 */

#ifndef ___CODER_CAM2WORLD_API_H__
#define ___CODER_CAM2WORLD_API_H__

/* Include Files */
#include "tmwtypes.h"
#include "mex.h"
#include "emlrt.h"
#include <stddef.h>
#include <stdlib.h>
#include "_coder_cam2world_api.h"

/* Variable Declarations */
extern emlrtCTX emlrtRootTLSGlobal;
extern emlrtContext emlrtContextGlobal;

/* Function Declarations */
extern void cam2world(real_T x, real_T y, real_T M[3]);
extern void cam2world_api(const mxArray * const prhs[2], const mxArray *plhs[1]);
extern void cam2world_atexit(void);
extern void cam2world_initialize(void);
extern void cam2world_terminate(void);
extern void cam2world_xil_terminate(void);

#endif

/*
 * File trailer for _coder_cam2world_api.h
 *
 * [EOF]
 */
