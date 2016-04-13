/*
 * Academic License - for use in teaching, academic research, and meeting
 * course requirements at degree granting institutions only.  Not for
 * government, commercial, or other organizational use.
 * File: _coder_world2cam_api.h
 *
 * MATLAB Coder version            : 2.8.1
 * C/C++ source code generated on  : 13-Mar-2016 13:50:05
 */

#ifndef ___CODER_WORLD2CAM_API_H__
#define ___CODER_WORLD2CAM_API_H__

/* Include Files */
#include "tmwtypes.h"
#include "mex.h"
#include "emlrt.h"
#include <stddef.h>
#include <stdlib.h>
#include "_coder_world2cam_api.h"

/* Variable Declarations */
extern emlrtCTX emlrtRootTLSGlobal;
extern emlrtContext emlrtContextGlobal;

/* Function Declarations */
extern void world2cam(real_T x, real_T y, real_T z, real_T m[2]);
extern void world2cam_api(const mxArray * const prhs[3], const mxArray *plhs[1]);
extern void world2cam_atexit(void);
extern void world2cam_initialize(void);
extern void world2cam_terminate(void);
extern void world2cam_xil_terminate(void);

#endif

/*
 * File trailer for _coder_world2cam_api.h
 *
 * [EOF]
 */
