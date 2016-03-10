/*
 * Academic License - for use in teaching, academic research, and meeting
 * course requirements at degree granting institutions only.  Not for
 * government, commercial, or other organizational use.
 * File: world2cam.c
 *
 * MATLAB Coder version            : 2.8.1
 * C/C++ source code generated on  : 10-Mar-2016 15:25:33
 */

/* Include Files */
#include "rt_nonfinite.h"
#include "world2cam.h"
#include "roots.h"
#include "world2cam_rtwutil.h"

/* Function Declarations */
static boolean_T eml_relop(const creal_T a, const creal_T b);
static double rt_atan2d_snf(double u0, double u1);

/* Function Definitions */

/*
 * Arguments    : const creal_T a
 *                const creal_T b
 * Return Type  : boolean_T
 */
static boolean_T eml_relop(const creal_T a, const creal_T b)
{
  boolean_T p;
  double absbi;
  double y;
  double absxk;
  int exponent;
  double absar;
  double absbr;
  double Ma;
  int b_exponent;
  int c_exponent;
  int d_exponent;
  if ((fabs(a.re) > 8.9884656743115785E+307) || (fabs(a.im) >
       8.9884656743115785E+307) || (fabs(b.re) > 8.9884656743115785E+307) ||
      (fabs(b.im) > 8.9884656743115785E+307)) {
    absbi = rt_hypotd_snf(a.re / 2.0, a.im / 2.0);
    y = rt_hypotd_snf(b.re / 2.0, b.im / 2.0);
  } else {
    absbi = rt_hypotd_snf(a.re, a.im);
    y = rt_hypotd_snf(b.re, b.im);
  }

  absxk = y / 2.0;
  if ((!rtIsInf(absxk)) && (!rtIsNaN(absxk))) {
    if (absxk <= 2.2250738585072014E-308) {
      absxk = 4.94065645841247E-324;
    } else {
      frexp(absxk, &exponent);
      absxk = ldexp(1.0, exponent - 53);
    }
  } else {
    absxk = rtNaN;
  }

  if ((fabs(y - absbi) < absxk) || (rtIsInf(absbi) && rtIsInf(y) && ((absbi >
         0.0) == (y > 0.0)))) {
    p = true;
  } else {
    p = false;
  }

  if (p) {
    absar = fabs(a.re);
    absxk = fabs(a.im);
    absbr = fabs(b.re);
    absbi = fabs(b.im);
    if (absar > absxk) {
      Ma = absar;
      absar = absxk;
    } else {
      Ma = absxk;
    }

    if (absbr > absbi) {
      absxk = absbr;
      absbr = absbi;
    } else {
      absxk = absbi;
    }

    if (Ma > absxk) {
      if (absar < absbr) {
        absbi = Ma - absxk;
        y = (absar / 2.0 + absbr / 2.0) / (Ma / 2.0 + absxk / 2.0) * (absbr -
          absar);
      } else {
        absbi = Ma;
        y = absxk;
      }
    } else if (Ma < absxk) {
      if (absar > absbr) {
        y = absxk - Ma;
        absbi = (absar / 2.0 + absbr / 2.0) / (Ma / 2.0 + absxk / 2.0) * (absar
          - absbr);
      } else {
        absbi = Ma;
        y = absxk;
      }
    } else {
      absbi = absar;
      y = absbr;
    }

    absxk = fabs(y / 2.0);
    if ((!rtIsInf(absxk)) && (!rtIsNaN(absxk))) {
      if (absxk <= 2.2250738585072014E-308) {
        absxk = 4.94065645841247E-324;
      } else {
        frexp(absxk, &b_exponent);
        absxk = ldexp(1.0, b_exponent - 53);
      }
    } else {
      absxk = rtNaN;
    }

    if ((fabs(y - absbi) < absxk) || (rtIsInf(absbi) && rtIsInf(y) && ((absbi >
           0.0) == (y > 0.0)))) {
      p = true;
    } else {
      p = false;
    }

    if (p) {
      absbi = rt_atan2d_snf(a.im, a.re);
      y = rt_atan2d_snf(b.im, b.re);
      absxk = fabs(y / 2.0);
      if ((!rtIsInf(absxk)) && (!rtIsNaN(absxk))) {
        if (absxk <= 2.2250738585072014E-308) {
          absxk = 4.94065645841247E-324;
        } else {
          frexp(absxk, &c_exponent);
          absxk = ldexp(1.0, c_exponent - 53);
        }
      } else {
        absxk = rtNaN;
      }

      if ((fabs(y - absbi) < absxk) || (rtIsInf(absbi) && rtIsInf(y) && ((absbi >
             0.0) == (y > 0.0)))) {
        p = true;
      } else {
        p = false;
      }

      if (p) {
        if (absbi > 0.78539816339744828) {
          if (absbi > 2.3561944901923448) {
            absbi = -a.im;
            y = -b.im;
          } else {
            absbi = -a.re;
            y = -b.re;
          }
        } else if (absbi > -0.78539816339744828) {
          absbi = a.im;
          y = b.im;
        } else if (absbi > -2.3561944901923448) {
          absbi = a.re;
          y = b.re;
        } else {
          absbi = -a.im;
          y = -b.im;
        }

        absxk = fabs(y / 2.0);
        if ((!rtIsInf(absxk)) && (!rtIsNaN(absxk))) {
          if (absxk <= 2.2250738585072014E-308) {
            absxk = 4.94065645841247E-324;
          } else {
            frexp(absxk, &d_exponent);
            absxk = ldexp(1.0, d_exponent - 53);
          }
        } else {
          absxk = rtNaN;
        }

        if ((fabs(y - absbi) < absxk) || (rtIsInf(absbi) && rtIsInf(y) &&
             ((absbi > 0.0) == (y > 0.0)))) {
          p = true;
        } else {
          p = false;
        }

        if (p) {
          absbi = 0.0;
          y = 0.0;
        }
      }
    }
  }

  return absbi < y;
}

/*
 * Arguments    : double u0
 *                double u1
 * Return Type  : double
 */
static double rt_atan2d_snf(double u0, double u1)
{
  double y;
  int b_u0;
  int b_u1;
  if (rtIsNaN(u0) || rtIsNaN(u1)) {
    y = rtNaN;
  } else if (rtIsInf(u0) && rtIsInf(u1)) {
    if (u0 > 0.0) {
      b_u0 = 1;
    } else {
      b_u0 = -1;
    }

    if (u1 > 0.0) {
      b_u1 = 1;
    } else {
      b_u1 = -1;
    }

    y = atan2(b_u0, b_u1);
  } else if (u1 == 0.0) {
    if (u0 > 0.0) {
      y = RT_PI / 2.0;
    } else if (u0 < 0.0) {
      y = -(double)(RT_PI / 2.0);
    } else {
      y = 0.0;
    }
  } else {
    y = atan2(u0, u1);
  }

  return y;
}

/*
 * Arguments    : double x
 *                double y
 *                double z
 *                double m[2]
 * Return Type  : void
 */
void world2cam(double x, double y, double z, double m[2])
{
  double M_idx_0;
  double M_idx_1;
  int ii_size_idx_0;
  int nx;
  int idx;
  int ii;
  double poly_coef_tmp[4];
  static const double poly_coef[4] = { 1.913645310011797E-5, 0.002319848981658,
    0.0, -134.29889751568481 };

  int rhoTmp_size[1];
  creal_T rhoTmp_data[3];
  boolean_T x_data[3];
  int ii_data[3];
  boolean_T exitg2;
  boolean_T guard1 = false;
  creal_T res_data[3];
  creal_T mtmp;
  boolean_T exitg1;
  double b_z;

  /* WORLD2CAM projects a 3D point on to the image */
  /*    m=WORLD2CAM(M, ocam_model) projects a 3D point on to the */
  /*    image and returns the pixel coordinates. */
  /*     */
  /*    M is a 3xN matrix containing the coordinates of the 3D points: M=[X;Y;Z] */
  /*    "ocam_model" contains the model of the calibrated camera. */
  /*    m=[rows;cols] is a 2xN matrix containing the returned rows and columns of the points after being */
  /*    reproject onto the image. */
  /*     */
  /*    Copyright (C) 2006 DAVIDE SCARAMUZZA    */
  /*    Author: Davide Scaramuzza - email: davide.scaramuzza@ieee.org */
  M_idx_0 = x;
  M_idx_1 = y;

  /* convert 3D coordinates vector into 2D pixel coordinates */
  /* These three lines overcome problem when xx = [0,0,+-1] */
  if ((x == 0.0) && (y == 0.0)) {
    ii_size_idx_0 = 1;
    nx = 1;
  } else {
    ii_size_idx_0 = 0;
    nx = 0;
  }

  idx = ii_size_idx_0 * nx;
  ii = 0;
  while (ii <= idx - 1) {
    M_idx_0 = 2.2204460492503131E-16;
    ii = 1;
  }

  idx = ii_size_idx_0 * nx;
  ii = 0;
  while (ii <= idx - 1) {
    M_idx_1 = 2.2204460492503131E-16;
    ii = 1;
  }

  for (nx = 0; nx < 4; nx++) {
    poly_coef_tmp[nx] = poly_coef[nx];
  }

  poly_coef_tmp[2] = 0.0 - z / sqrt(M_idx_0 * M_idx_0 + M_idx_1 * M_idx_1);
  roots(poly_coef_tmp, rhoTmp_data, rhoTmp_size);
  idx = rhoTmp_size[0];
  for (ii = 0; ii < idx; ii++) {
    x_data[ii] = ((rhoTmp_data[ii].im == 0.0) && (rhoTmp_data[ii].re > 0.0));
  }

  nx = rhoTmp_size[0];
  idx = 0;
  ii_size_idx_0 = rhoTmp_size[0];
  ii = 1;
  exitg2 = false;
  while ((!exitg2) && (ii <= nx)) {
    guard1 = false;
    if (x_data[ii - 1]) {
      idx++;
      ii_data[idx - 1] = ii;
      if (idx >= nx) {
        exitg2 = true;
      } else {
        guard1 = true;
      }
    } else {
      guard1 = true;
    }

    if (guard1) {
      ii++;
    }
  }

  if (rhoTmp_size[0] == 1) {
    if (idx == 0) {
      ii_size_idx_0 = 0;
    }
  } else if (1 > idx) {
    ii_size_idx_0 = 0;
  } else {
    ii_size_idx_0 = idx;
  }

  for (ii = 0; ii < ii_size_idx_0; ii++) {
    res_data[ii] = rhoTmp_data[ii_data[ii] - 1];
  }

  /*  & rhoTmp<height ));    %obrand */
  if (ii_size_idx_0 == 0) {
    /* | length(res)>1    %obrand */
    ii_size_idx_0 = 1;
    res_data[0].re = rtNaN;
    res_data[0].im = 0.0;
  } else {
    if (ii_size_idx_0 > 1) {
      /* obrand */
      nx = 1;
      mtmp = res_data[0];
      if (rtIsNaN(res_data[0].re) || rtIsNaN(res_data[0].im)) {
        idx = 1;
        exitg1 = false;
        while ((!exitg1) && (idx + 1 <= ii_size_idx_0)) {
          nx = idx + 1;
          if (!(rtIsNaN(res_data[idx].re) || rtIsNaN(res_data[idx].im))) {
            mtmp = res_data[idx];
            exitg1 = true;
          } else {
            idx++;
          }
        }
      }

      if (nx < ii_size_idx_0) {
        while (nx + 1 <= ii_size_idx_0) {
          if (eml_relop(res_data[nx], mtmp)) {
            mtmp = res_data[nx];
          }

          nx++;
        }
      }

      ii_size_idx_0 = 1;
      res_data[0] = mtmp;

      /* obrand */
    }
  }

  b_z = M_idx_0 / sqrt(M_idx_0 * M_idx_0 + M_idx_1 * M_idx_1);
  for (ii = 0; ii < ii_size_idx_0; ii++) {
    rhoTmp_data[ii].re = b_z * res_data[ii].re;
    rhoTmp_data[ii].im = b_z * res_data[ii].im;
  }

  b_z = M_idx_1 / sqrt(M_idx_0 * M_idx_0 + M_idx_1 * M_idx_1);
  for (ii = 0; ii < ii_size_idx_0; ii++) {
    res_data[ii].re *= b_z;
    res_data[ii].im *= b_z;
  }

  m[0] = (rhoTmp_data[0].re * 1.00812969679143 + res_data[0].re *
          0.005332214439608) + 136.1364707499628;
  m[1] = (rhoTmp_data[0].re * 0.018209638073116 + res_data[0].re) +
    120.0594502378968;
}

/*
 * File trailer for world2cam.c
 *
 * [EOF]
 */
