/*
 * Academic License - for use in teaching, academic research, and meeting
 * course requirements at degree granting institutions only.  Not for
 * government, commercial, or other organizational use.
 * File: cam2world.c
 *
 * MATLAB Coder version            : 2.8.1
 * C/C++ source code generated on  : 11-Mar-2016 16:57:34
 */

/* Include Files */
#include "rt_nonfinite.h"
#include "cam2world.h"

/* Function Definitions */

/*
 * Arguments    : double x
 *                double y
 *                double M[3]
 * Return Type  : void
 */
void cam2world(double x, double y, double M[3])
{
  double b_x[2];
  double c_x[2];
  int k;
  double m[2];
  int i0;
  static const double a[4] = { 0.99203140939731782, -0.018064532922288327,
    -0.0052897242057330538, 1.000096323963293 };

  double d_x;
  double b_y;
  static const double p[4] = { 1.913645310011797E-5, 0.002319848981658, 0.0,
    -134.29889751568481 };

  /* CAM2WORLD Project a give pixel point onto the unit sphere */
  /*    M=CAM2WORLD=(m, ocam_model) returns the 3D coordinates of the vector */
  /*    emanating from the single effective viewpoint on the unit sphere */
  /*  */
  /*    m=[rows;cols] is a 2xN matrix containing the pixel coordinates of the image */
  /*    points. */
  /*  */
  /*    "ocam_model" contains the model of the calibrated camera. */
  /*     */
  /*    M=[X;Y;Z] is a 3xN matrix with the coordinates on the unit sphere: */
  /*    thus, X^2 + Y^2 + Z^2 = 1 */
  /*     */
  /*    Last update May 2009 */
  /*    Copyright (C) 2006 DAVIDE SCARAMUZZA    */
  /*    Author: Davide Scaramuzza - email: davide.scaramuzza@ieee.org */
  b_x[0] = x;
  b_x[1] = y;
  for (k = 0; k < 2; k++) {
    c_x[k] = b_x[k] - (136.1364707499628 + -16.077020512066 * (double)k);
  }

  for (k = 0; k < 2; k++) {
    m[k] = 0.0;
    for (i0 = 0; i0 < 2; i0++) {
      m[k] += a[k + (i0 << 1)] * c_x[i0];
    }
  }

  /*  Given an image point it returns the 3D coordinates of its correspondent optical */
  /*  ray */
  d_x = sqrt(m[0] * m[0] + m[1] * m[1]);
  b_y = 1.913645310011797E-5;
  for (k = 0; k < 3; k++) {
    b_y = d_x * b_y + p[k + 1];
  }

  M[0] = m[0];
  M[1] = m[1];
  M[2] = b_y;
}

/*
 * File trailer for cam2world.c
 *
 * [EOF]
 */
