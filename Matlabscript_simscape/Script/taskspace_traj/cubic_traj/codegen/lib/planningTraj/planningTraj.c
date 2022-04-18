/*
 * Academic License - for use in teaching, academic research, and meeting
 * course requirements at degree granting institutions only.  Not for
 * government, commercial, or other organizational use.
 * File: planningTraj.c
 *
 * MATLAB Coder version            : 5.3
 * C/C++ source code generated on  : 18-Apr-2022 13:57:29
 */

/* Include Files */
#include "planningTraj.h"
#include "planningTraj_emxutil.h"
#include "planningTraj_types.h"
#include "rt_nonfinite.h"
#include "rt_nonfinite.h"
#include <math.h>

/* Function Declarations */
static void b_binary_expand_op(emxArray_real_T *q, double a, double b_a,
                               double q0, double y, double c_a,
                               const emxArray_real_T *v, double tf, double d_a,
                               const emxArray_real_T *t, double b, double v0,
                               double v1, double b_b, double c_b, double c);

static void binary_expand_op(emxArray_real_T *v, double a, double b_a, double y,
                             double v0, const emxArray_real_T *t, double b,
                             double tf, double b_b, double v1, double c_a,
                             double d_a, double b_y);

static double rt_powd_snf(double u0, double u1);

/* Function Definitions */
/*
 * Arguments    : emxArray_real_T *q
 *                double a
 *                double b_a
 *                double q0
 *                double y
 *                double c_a
 *                const emxArray_real_T *v
 *                double tf
 *                double d_a
 *                const emxArray_real_T *t
 *                double b
 *                double v0
 *                double v1
 *                double b_b
 *                double c_b
 *                double c
 * Return Type  : void
 */
static void b_binary_expand_op(emxArray_real_T *q, double a, double b_a,
                               double q0, double y, double c_a,
                               const emxArray_real_T *v, double tf, double d_a,
                               const emxArray_real_T *t, double b, double v0,
                               double v1, double b_b, double c_b, double c)
{
  emxArray_real_T *e_a;
  const double *t_data;
  const double *v_data;
  double b_q0;
  double *a_data;
  double *q_data;
  int i;
  int loop_ub;
  int stride_0_1;
  int stride_1_1;
  int stride_2_1;
  int stride_3_1;
  int stride_4_1;
  int stride_5_1;
  int stride_6_1;
  int stride_7_1;
  int stride_8_1;
  t_data = t->data;
  v_data = v->data;
  q_data = q->data;
  emxInit_real_T(&e_a, 2);
  b_q0 = q0 * y;
  i = e_a->size[0] * e_a->size[1];
  e_a->size[0] = 1;
  if (v->size[1] == 1) {
    if (q->size[1] == 1) {
      if (t->size[1] == 1) {
        if (v->size[1] == 1) {
          e_a->size[1] = q->size[1];
        } else {
          e_a->size[1] = v->size[1];
        }
      } else {
        e_a->size[1] = t->size[1];
      }
    } else {
      e_a->size[1] = q->size[1];
    }
  } else {
    e_a->size[1] = v->size[1];
  }
  emxEnsureCapacity_real_T(e_a, i);
  a_data = e_a->data;
  stride_0_1 = (q->size[1] != 1);
  stride_1_1 = (q->size[1] != 1);
  stride_2_1 = (v->size[1] != 1);
  stride_3_1 = (v->size[1] != 1);
  stride_4_1 = (t->size[1] != 1);
  stride_5_1 = (q->size[1] != 1);
  stride_6_1 = (q->size[1] != 1);
  stride_7_1 = (v->size[1] != 1);
  stride_8_1 = (v->size[1] != 1);
  if (v->size[1] == 1) {
    if (q->size[1] == 1) {
      if (t->size[1] == 1) {
        if (v->size[1] == 1) {
          loop_ub = q->size[1];
        } else {
          loop_ub = v->size[1];
        }
      } else {
        loop_ub = t->size[1];
      }
    } else {
      loop_ub = q->size[1];
    }
  } else {
    loop_ub = v->size[1];
  }
  for (i = 0; i < loop_ub; i++) {
    a_data[i] =
        (((((((((a * q_data[i * stride_0_1] - b_a * q_data[i * stride_1_1]) +
                b_q0) -
               c_a * v_data[i * stride_2_1] * tf) +
              d_a * v_data[i * stride_3_1] * tf) +
             t_data[i * stride_4_1] * b * v0) +
            q_data[i * stride_5_1] * tf * v0) +
           q_data[i * stride_6_1] * tf * v1) -
          2.0 * v_data[i * stride_7_1] * b_b * v0) -
         v_data[i * stride_8_1] * c_b * v1) /
        c;
  }
  i = q->size[0] * q->size[1];
  q->size[0] = 1;
  q->size[1] = e_a->size[1];
  emxEnsureCapacity_real_T(q, i);
  q_data = q->data;
  loop_ub = e_a->size[1];
  for (i = 0; i < loop_ub; i++) {
    q_data[i] = a_data[i];
  }
  emxFree_real_T(&e_a);
}

/*
 * Arguments    : emxArray_real_T *v
 *                double a
 *                double b_a
 *                double y
 *                double v0
 *                const emxArray_real_T *t
 *                double b
 *                double tf
 *                double b_b
 *                double v1
 *                double c_a
 *                double d_a
 *                double b_y
 * Return Type  : void
 */
static void binary_expand_op(emxArray_real_T *v, double a, double b_a, double y,
                             double v0, const emxArray_real_T *t, double b,
                             double tf, double b_b, double v1, double c_a,
                             double d_a, double b_y)
{
  emxArray_real_T *e_a;
  const double *t_data;
  double c_y;
  double *a_data;
  double *v_data;
  int i;
  int loop_ub;
  int stride_0_1;
  int stride_1_1;
  int stride_2_1;
  int stride_3_1;
  int stride_4_1;
  int stride_5_1;
  int stride_6_1;
  int stride_7_1;
  t_data = t->data;
  v_data = v->data;
  emxInit_real_T(&e_a, 2);
  c_y = y * v0;
  i = e_a->size[0] * e_a->size[1];
  e_a->size[0] = 1;
  if (t->size[1] == 1) {
    if (v->size[1] == 1) {
      if (t->size[1] == 1) {
        if (v->size[1] == 1) {
          if (t->size[1] == 1) {
            e_a->size[1] = v->size[1];
          } else {
            e_a->size[1] = t->size[1];
          }
        } else {
          e_a->size[1] = v->size[1];
        }
      } else {
        e_a->size[1] = t->size[1];
      }
    } else {
      e_a->size[1] = v->size[1];
    }
  } else {
    e_a->size[1] = t->size[1];
  }
  emxEnsureCapacity_real_T(e_a, i);
  a_data = e_a->data;
  stride_0_1 = (v->size[1] != 1);
  stride_1_1 = (v->size[1] != 1);
  stride_2_1 = (t->size[1] != 1);
  stride_3_1 = (v->size[1] != 1);
  stride_4_1 = (t->size[1] != 1);
  stride_5_1 = (v->size[1] != 1);
  stride_6_1 = (t->size[1] != 1);
  stride_7_1 = (t->size[1] != 1);
  if (t->size[1] == 1) {
    if (v->size[1] == 1) {
      if (t->size[1] == 1) {
        if (v->size[1] == 1) {
          if (t->size[1] == 1) {
            loop_ub = v->size[1];
          } else {
            loop_ub = t->size[1];
          }
        } else {
          loop_ub = v->size[1];
        }
      } else {
        loop_ub = t->size[1];
      }
    } else {
      loop_ub = v->size[1];
    }
  } else {
    loop_ub = t->size[1];
  }
  for (i = 0; i < loop_ub; i++) {
    a_data[i] =
        ((((((((a * v_data[i * stride_0_1] - b_a * v_data[i * stride_1_1]) +
               c_y) -
              4.0 * t_data[i * stride_2_1] * b * v0) +
             3.0 * v_data[i * stride_3_1] * tf * v0) -
            2.0 * t_data[i * stride_4_1] * b_b * v1) +
           3.0 * v_data[i * stride_5_1] * tf * v1) -
          c_a * t_data[i * stride_6_1] * tf) +
         d_a * t_data[i * stride_7_1] * tf) /
        b_y;
  }
  i = v->size[0] * v->size[1];
  v->size[0] = 1;
  v->size[1] = e_a->size[1];
  emxEnsureCapacity_real_T(v, i);
  v_data = v->data;
  loop_ub = e_a->size[1];
  for (i = 0; i < loop_ub; i++) {
    v_data[i] = a_data[i];
  }
  emxFree_real_T(&e_a);
}

/*
 * Arguments    : double u0
 *                double u1
 * Return Type  : double
 */
static double rt_powd_snf(double u0, double u1)
{
  double d;
  double d1;
  double y;
  if (rtIsNaN(u0) || rtIsNaN(u1)) {
    y = rtNaN;
  } else {
    d = fabs(u0);
    d1 = fabs(u1);
    if (rtIsInf(u1)) {
      if (d == 1.0) {
        y = 1.0;
      } else if (d > 1.0) {
        if (u1 > 0.0) {
          y = rtInf;
        } else {
          y = 0.0;
        }
      } else if (u1 > 0.0) {
        y = 0.0;
      } else {
        y = rtInf;
      }
    } else if (d1 == 0.0) {
      y = 1.0;
    } else if (d1 == 1.0) {
      if (u1 > 0.0) {
        y = u0;
      } else {
        y = 1.0 / u0;
      }
    } else if (u1 == 2.0) {
      y = u0 * u0;
    } else if ((u1 == 0.5) && (u0 >= 0.0)) {
      y = sqrt(u0);
    } else if ((u0 < 0.0) && (u1 > floor(u1))) {
      y = rtNaN;
    } else {
      y = pow(u0, u1);
    }
  }
  return y;
}

/*
 * Arguments    : double q0
 *                double q1
 *                double v0
 *                double v1
 *                double tf
 *                emxArray_real_T *q
 *                emxArray_real_T *v
 * Return Type  : void
 */
void planningTraj(double q0, double q1, double v0, double v1, double tf,
                  emxArray_real_T *q, emxArray_real_T *v)
{
  emxArray_real_T *t;
  double a;
  double b_a;
  double b_q0;
  double b_tmp;
  double c_a;
  double d;
  double d1;
  double d2;
  double delta1;
  double y_tmp;
  double *q_data;
  double *t_data;
  double *v_data;
  int b_q;
  int c_q;
  int d_q;
  int e_q;
  int f_q;
  int g_q;
  int h_q;
  int i_q;
  int k;
  int t_tmp_tmp;
  delta1 = 1000.0 * tf;
  emxInit_real_T(&t, 2);
  t_data = t->data;
  if (!(delta1 >= 0.0)) {
    t->size[0] = 1;
    t->size[1] = 0;
  } else {
    d = floor(delta1);
    b_q = t->size[0] * t->size[1];
    t->size[0] = 1;
    t->size[1] = (int)d;
    emxEnsureCapacity_real_T(t, b_q);
    t_data = t->data;
    if ((int)d >= 1) {
      t_tmp_tmp = (int)d - 1;
      t_data[(int)floor(delta1) - 1] = tf;
      if (t->size[1] >= 2) {
        t_data[0] = 0.0;
        if (t->size[1] >= 3) {
          if ((0.0 == -tf) && ((int)d > 2)) {
            delta1 = tf / ((double)(int)d - 1.0);
            for (k = 2; k <= t_tmp_tmp; k++) {
              t_data[k - 1] = (double)(((k << 1) - (int)d) - 1) * delta1;
            }
            if (((int)d & 1) == 1) {
              t_data[(int)d >> 1] = 0.0;
            }
          } else if ((tf < 0.0) && (fabs(tf) > 8.9884656743115785E+307)) {
            delta1 = tf / ((double)t->size[1] - 1.0);
            b_q = t->size[1];
            for (k = 0; k <= b_q - 3; k++) {
              t_data[k + 1] = delta1 * ((double)k + 1.0);
            }
          } else {
            delta1 = tf / ((double)t->size[1] - 1.0);
            b_q = t->size[1];
            for (k = 0; k <= b_q - 3; k++) {
              t_data[k + 1] = ((double)k + 1.0) * delta1;
            }
          }
        }
      }
    }
  }
  a = 2.0 * q0;
  b_q = q->size[0] * q->size[1];
  q->size[0] = 1;
  q->size[1] = t->size[1];
  emxEnsureCapacity_real_T(q, b_q);
  q_data = q->data;
  t_tmp_tmp = t->size[1];
  for (b_q = 0; b_q < t_tmp_tmp; b_q++) {
    delta1 = t_data[b_q];
    q_data[b_q] = rt_powd_snf(delta1, 3.0);
  }
  b_a = 2.0 * q1;
  c_a = 3.0 * q0;
  b_q = v->size[0] * v->size[1];
  v->size[0] = 1;
  v->size[1] = t->size[1];
  emxEnsureCapacity_real_T(v, b_q);
  v_data = v->data;
  t_tmp_tmp = t->size[1];
  for (b_q = 0; b_q < t_tmp_tmp; b_q++) {
    delta1 = t_data[b_q];
    v_data[b_q] = delta1 * delta1;
  }
  y_tmp = rt_powd_snf(tf, 3.0);
  delta1 = 3.0 * q1;
  b_tmp = tf * tf;
  if (q->size[1] == 1) {
    t_tmp_tmp = v->size[1];
  } else {
    t_tmp_tmp = q->size[1];
  }
  if (q->size[1] == 1) {
    b_q = v->size[1];
  } else {
    b_q = q->size[1];
  }
  if (b_q == 1) {
    b_q = v->size[1];
  } else if (q->size[1] == 1) {
    b_q = v->size[1];
  } else {
    b_q = q->size[1];
  }
  if (q->size[1] == 1) {
    c_q = v->size[1];
  } else {
    c_q = q->size[1];
  }
  if (c_q == 1) {
    c_q = v->size[1];
  } else if (q->size[1] == 1) {
    c_q = v->size[1];
  } else {
    c_q = q->size[1];
  }
  if (q->size[1] == 1) {
    d_q = v->size[1];
  } else {
    d_q = q->size[1];
  }
  if (c_q == 1) {
    c_q = t->size[1];
  } else if (d_q == 1) {
    c_q = v->size[1];
  } else if (q->size[1] == 1) {
    c_q = v->size[1];
  } else {
    c_q = q->size[1];
  }
  if (q->size[1] == 1) {
    d_q = v->size[1];
  } else {
    d_q = q->size[1];
  }
  if (d_q == 1) {
    d_q = v->size[1];
  } else if (q->size[1] == 1) {
    d_q = v->size[1];
  } else {
    d_q = q->size[1];
  }
  if (q->size[1] == 1) {
    e_q = v->size[1];
  } else {
    e_q = q->size[1];
  }
  if (d_q == 1) {
    d_q = t->size[1];
  } else if (e_q == 1) {
    d_q = v->size[1];
  } else if (q->size[1] == 1) {
    d_q = v->size[1];
  } else {
    d_q = q->size[1];
  }
  if (q->size[1] == 1) {
    e_q = v->size[1];
  } else {
    e_q = q->size[1];
  }
  if (e_q == 1) {
    e_q = v->size[1];
  } else if (q->size[1] == 1) {
    e_q = v->size[1];
  } else {
    e_q = q->size[1];
  }
  if (q->size[1] == 1) {
    f_q = v->size[1];
  } else {
    f_q = q->size[1];
  }
  if (d_q == 1) {
    d_q = q->size[1];
  } else if (e_q == 1) {
    d_q = t->size[1];
  } else if (f_q == 1) {
    d_q = v->size[1];
  } else if (q->size[1] == 1) {
    d_q = v->size[1];
  } else {
    d_q = q->size[1];
  }
  if (q->size[1] == 1) {
    e_q = v->size[1];
  } else {
    e_q = q->size[1];
  }
  if (e_q == 1) {
    e_q = v->size[1];
  } else if (q->size[1] == 1) {
    e_q = v->size[1];
  } else {
    e_q = q->size[1];
  }
  if (q->size[1] == 1) {
    f_q = v->size[1];
  } else {
    f_q = q->size[1];
  }
  if (e_q == 1) {
    e_q = t->size[1];
  } else if (f_q == 1) {
    e_q = v->size[1];
  } else if (q->size[1] == 1) {
    e_q = v->size[1];
  } else {
    e_q = q->size[1];
  }
  if (q->size[1] == 1) {
    f_q = v->size[1];
  } else {
    f_q = q->size[1];
  }
  if (f_q == 1) {
    f_q = v->size[1];
  } else if (q->size[1] == 1) {
    f_q = v->size[1];
  } else {
    f_q = q->size[1];
  }
  if (q->size[1] == 1) {
    g_q = v->size[1];
  } else {
    g_q = q->size[1];
  }
  if (e_q == 1) {
    e_q = q->size[1];
  } else if (f_q == 1) {
    e_q = t->size[1];
  } else if (g_q == 1) {
    e_q = v->size[1];
  } else if (q->size[1] == 1) {
    e_q = v->size[1];
  } else {
    e_q = q->size[1];
  }
  if (q->size[1] == 1) {
    f_q = v->size[1];
  } else {
    f_q = q->size[1];
  }
  if (f_q == 1) {
    f_q = v->size[1];
  } else if (q->size[1] == 1) {
    f_q = v->size[1];
  } else {
    f_q = q->size[1];
  }
  if (q->size[1] == 1) {
    g_q = v->size[1];
  } else {
    g_q = q->size[1];
  }
  if (f_q == 1) {
    f_q = t->size[1];
  } else if (g_q == 1) {
    f_q = v->size[1];
  } else if (q->size[1] == 1) {
    f_q = v->size[1];
  } else {
    f_q = q->size[1];
  }
  if (q->size[1] == 1) {
    g_q = v->size[1];
  } else {
    g_q = q->size[1];
  }
  if (g_q == 1) {
    g_q = v->size[1];
  } else if (q->size[1] == 1) {
    g_q = v->size[1];
  } else {
    g_q = q->size[1];
  }
  if (q->size[1] == 1) {
    k = v->size[1];
  } else {
    k = q->size[1];
  }
  if (e_q == 1) {
    e_q = q->size[1];
  } else if (f_q == 1) {
    e_q = q->size[1];
  } else if (g_q == 1) {
    e_q = t->size[1];
  } else if (k == 1) {
    e_q = v->size[1];
  } else if (q->size[1] == 1) {
    e_q = v->size[1];
  } else {
    e_q = q->size[1];
  }
  if (q->size[1] == 1) {
    f_q = v->size[1];
  } else {
    f_q = q->size[1];
  }
  if (f_q == 1) {
    f_q = v->size[1];
  } else if (q->size[1] == 1) {
    f_q = v->size[1];
  } else {
    f_q = q->size[1];
  }
  if (q->size[1] == 1) {
    g_q = v->size[1];
  } else {
    g_q = q->size[1];
  }
  if (f_q == 1) {
    f_q = t->size[1];
  } else if (g_q == 1) {
    f_q = v->size[1];
  } else if (q->size[1] == 1) {
    f_q = v->size[1];
  } else {
    f_q = q->size[1];
  }
  if (q->size[1] == 1) {
    g_q = v->size[1];
  } else {
    g_q = q->size[1];
  }
  if (g_q == 1) {
    g_q = v->size[1];
  } else if (q->size[1] == 1) {
    g_q = v->size[1];
  } else {
    g_q = q->size[1];
  }
  if (q->size[1] == 1) {
    k = v->size[1];
  } else {
    k = q->size[1];
  }
  if (f_q == 1) {
    f_q = q->size[1];
  } else if (g_q == 1) {
    f_q = t->size[1];
  } else if (k == 1) {
    f_q = v->size[1];
  } else if (q->size[1] == 1) {
    f_q = v->size[1];
  } else {
    f_q = q->size[1];
  }
  if (q->size[1] == 1) {
    g_q = v->size[1];
  } else {
    g_q = q->size[1];
  }
  if (g_q == 1) {
    g_q = v->size[1];
  } else if (q->size[1] == 1) {
    g_q = v->size[1];
  } else {
    g_q = q->size[1];
  }
  if (q->size[1] == 1) {
    k = v->size[1];
  } else {
    k = q->size[1];
  }
  if (g_q == 1) {
    g_q = t->size[1];
  } else if (k == 1) {
    g_q = v->size[1];
  } else if (q->size[1] == 1) {
    g_q = v->size[1];
  } else {
    g_q = q->size[1];
  }
  if (q->size[1] == 1) {
    k = v->size[1];
  } else {
    k = q->size[1];
  }
  if (k == 1) {
    k = v->size[1];
  } else if (q->size[1] == 1) {
    k = v->size[1];
  } else {
    k = q->size[1];
  }
  if (q->size[1] == 1) {
    h_q = v->size[1];
  } else {
    h_q = q->size[1];
  }
  if (f_q == 1) {
    f_q = q->size[1];
  } else if (g_q == 1) {
    f_q = q->size[1];
  } else if (k == 1) {
    f_q = t->size[1];
  } else if (h_q == 1) {
    f_q = v->size[1];
  } else if (q->size[1] == 1) {
    f_q = v->size[1];
  } else {
    f_q = q->size[1];
  }
  if (q->size[1] == 1) {
    g_q = v->size[1];
  } else {
    g_q = q->size[1];
  }
  if (g_q == 1) {
    g_q = v->size[1];
  } else if (q->size[1] == 1) {
    g_q = v->size[1];
  } else {
    g_q = q->size[1];
  }
  if (q->size[1] == 1) {
    k = v->size[1];
  } else {
    k = q->size[1];
  }
  if (g_q == 1) {
    g_q = t->size[1];
  } else if (k == 1) {
    g_q = v->size[1];
  } else if (q->size[1] == 1) {
    g_q = v->size[1];
  } else {
    g_q = q->size[1];
  }
  if (q->size[1] == 1) {
    k = v->size[1];
  } else {
    k = q->size[1];
  }
  if (k == 1) {
    k = v->size[1];
  } else if (q->size[1] == 1) {
    k = v->size[1];
  } else {
    k = q->size[1];
  }
  if (q->size[1] == 1) {
    h_q = v->size[1];
  } else {
    h_q = q->size[1];
  }
  if (g_q == 1) {
    g_q = q->size[1];
  } else if (k == 1) {
    g_q = t->size[1];
  } else if (h_q == 1) {
    g_q = v->size[1];
  } else if (q->size[1] == 1) {
    g_q = v->size[1];
  } else {
    g_q = q->size[1];
  }
  if (q->size[1] == 1) {
    k = v->size[1];
  } else {
    k = q->size[1];
  }
  if (k == 1) {
    k = v->size[1];
  } else if (q->size[1] == 1) {
    k = v->size[1];
  } else {
    k = q->size[1];
  }
  if (q->size[1] == 1) {
    h_q = v->size[1];
  } else {
    h_q = q->size[1];
  }
  if (k == 1) {
    k = t->size[1];
  } else if (h_q == 1) {
    k = v->size[1];
  } else if (q->size[1] == 1) {
    k = v->size[1];
  } else {
    k = q->size[1];
  }
  if (q->size[1] == 1) {
    h_q = v->size[1];
  } else {
    h_q = q->size[1];
  }
  if (h_q == 1) {
    h_q = v->size[1];
  } else if (q->size[1] == 1) {
    h_q = v->size[1];
  } else {
    h_q = q->size[1];
  }
  if (q->size[1] == 1) {
    i_q = v->size[1];
  } else {
    i_q = q->size[1];
  }
  if (f_q == 1) {
    f_q = v->size[1];
  } else if (g_q == 1) {
    f_q = q->size[1];
  } else if (k == 1) {
    f_q = q->size[1];
  } else if (h_q == 1) {
    f_q = t->size[1];
  } else if (i_q == 1) {
    f_q = v->size[1];
  } else if (q->size[1] == 1) {
    f_q = v->size[1];
  } else {
    f_q = q->size[1];
  }
  if ((q->size[1] == v->size[1]) && (t_tmp_tmp == v->size[1]) &&
      (b_q == t->size[1]) && (c_q == q->size[1]) && (d_q == q->size[1]) &&
      (e_q == v->size[1]) && (f_q == v->size[1])) {
    b_q0 = q0 * y_tmp;
    t_tmp_tmp = q->size[1] - 1;
    b_q = q->size[0] * q->size[1];
    q->size[0] = 1;
    emxEnsureCapacity_real_T(q, b_q);
    q_data = q->data;
    for (b_q = 0; b_q <= t_tmp_tmp; b_q++) {
      d = q_data[b_q];
      d1 = v_data[b_q];
      d2 = d * tf;
      d = (((((((((a * d - b_a * d) + b_q0) - c_a * d1 * tf) +
                delta1 * d1 * tf) +
               t_data[b_q] * y_tmp * v0) +
              d2 * v0) +
             d2 * v1) -
            2.0 * d1 * b_tmp * v0) -
           d1 * b_tmp * v1) /
          y_tmp;
      q_data[b_q] = d;
    }
  } else {
    b_binary_expand_op(q, a, b_a, q0, y_tmp, c_a, v, tf, delta1, t, y_tmp, v0,
                       v1, b_tmp, b_tmp, y_tmp);
  }
  delta1 = 6.0 * q0;
  b_q0 = 6.0 * q1;
  if (v->size[1] == 1) {
    t_tmp_tmp = t->size[1];
  } else {
    t_tmp_tmp = v->size[1];
  }
  if (v->size[1] == 1) {
    b_q = t->size[1];
  } else {
    b_q = v->size[1];
  }
  if (b_q == 1) {
    b_q = v->size[1];
  } else if (v->size[1] == 1) {
    b_q = t->size[1];
  } else {
    b_q = v->size[1];
  }
  if (v->size[1] == 1) {
    k = t->size[1];
  } else {
    k = v->size[1];
  }
  if (k == 1) {
    k = v->size[1];
  } else if (v->size[1] == 1) {
    k = t->size[1];
  } else {
    k = v->size[1];
  }
  if (v->size[1] == 1) {
    c_q = t->size[1];
  } else {
    c_q = v->size[1];
  }
  if (k == 1) {
    k = t->size[1];
  } else if (c_q == 1) {
    k = v->size[1];
  } else if (v->size[1] == 1) {
    k = t->size[1];
  } else {
    k = v->size[1];
  }
  if (v->size[1] == 1) {
    c_q = t->size[1];
  } else {
    c_q = v->size[1];
  }
  if (c_q == 1) {
    c_q = v->size[1];
  } else if (v->size[1] == 1) {
    c_q = t->size[1];
  } else {
    c_q = v->size[1];
  }
  if (v->size[1] == 1) {
    d_q = t->size[1];
  } else {
    d_q = v->size[1];
  }
  if (c_q == 1) {
    c_q = t->size[1];
  } else if (d_q == 1) {
    c_q = v->size[1];
  } else if (v->size[1] == 1) {
    c_q = t->size[1];
  } else {
    c_q = v->size[1];
  }
  if (v->size[1] == 1) {
    d_q = t->size[1];
  } else {
    d_q = v->size[1];
  }
  if (d_q == 1) {
    d_q = v->size[1];
  } else if (v->size[1] == 1) {
    d_q = t->size[1];
  } else {
    d_q = v->size[1];
  }
  if (v->size[1] == 1) {
    e_q = t->size[1];
  } else {
    e_q = v->size[1];
  }
  if (c_q == 1) {
    c_q = v->size[1];
  } else if (d_q == 1) {
    c_q = t->size[1];
  } else if (e_q == 1) {
    c_q = v->size[1];
  } else if (v->size[1] == 1) {
    c_q = t->size[1];
  } else {
    c_q = v->size[1];
  }
  if (v->size[1] == 1) {
    d_q = t->size[1];
  } else {
    d_q = v->size[1];
  }
  if (d_q == 1) {
    d_q = v->size[1];
  } else if (v->size[1] == 1) {
    d_q = t->size[1];
  } else {
    d_q = v->size[1];
  }
  if (v->size[1] == 1) {
    e_q = t->size[1];
  } else {
    e_q = v->size[1];
  }
  if (d_q == 1) {
    d_q = t->size[1];
  } else if (e_q == 1) {
    d_q = v->size[1];
  } else if (v->size[1] == 1) {
    d_q = t->size[1];
  } else {
    d_q = v->size[1];
  }
  if (v->size[1] == 1) {
    e_q = t->size[1];
  } else {
    e_q = v->size[1];
  }
  if (e_q == 1) {
    e_q = v->size[1];
  } else if (v->size[1] == 1) {
    e_q = t->size[1];
  } else {
    e_q = v->size[1];
  }
  if (v->size[1] == 1) {
    f_q = t->size[1];
  } else {
    f_q = v->size[1];
  }
  if (d_q == 1) {
    d_q = v->size[1];
  } else if (e_q == 1) {
    d_q = t->size[1];
  } else if (f_q == 1) {
    d_q = v->size[1];
  } else if (v->size[1] == 1) {
    d_q = t->size[1];
  } else {
    d_q = v->size[1];
  }
  if (v->size[1] == 1) {
    e_q = t->size[1];
  } else {
    e_q = v->size[1];
  }
  if (e_q == 1) {
    e_q = v->size[1];
  } else if (v->size[1] == 1) {
    e_q = t->size[1];
  } else {
    e_q = v->size[1];
  }
  if (v->size[1] == 1) {
    f_q = t->size[1];
  } else {
    f_q = v->size[1];
  }
  if (e_q == 1) {
    e_q = t->size[1];
  } else if (f_q == 1) {
    e_q = v->size[1];
  } else if (v->size[1] == 1) {
    e_q = t->size[1];
  } else {
    e_q = v->size[1];
  }
  if (v->size[1] == 1) {
    f_q = t->size[1];
  } else {
    f_q = v->size[1];
  }
  if (f_q == 1) {
    f_q = v->size[1];
  } else if (v->size[1] == 1) {
    f_q = t->size[1];
  } else {
    f_q = v->size[1];
  }
  if (v->size[1] == 1) {
    g_q = t->size[1];
  } else {
    g_q = v->size[1];
  }
  if (d_q == 1) {
    d_q = t->size[1];
  } else if (e_q == 1) {
    d_q = v->size[1];
  } else if (f_q == 1) {
    d_q = t->size[1];
  } else if (g_q == 1) {
    d_q = v->size[1];
  } else if (v->size[1] == 1) {
    d_q = t->size[1];
  } else {
    d_q = v->size[1];
  }
  if ((v->size[1] == t->size[1]) && (t_tmp_tmp == v->size[1]) &&
      (b_q == t->size[1]) && (k == v->size[1]) && (c_q == t->size[1]) &&
      (d_q == t->size[1])) {
    a = y_tmp * v0;
    t_tmp_tmp = v->size[1] - 1;
    b_q = v->size[0] * v->size[1];
    v->size[0] = 1;
    emxEnsureCapacity_real_T(v, b_q);
    v_data = v->data;
    for (b_q = 0; b_q <= t_tmp_tmp; b_q++) {
      d = v_data[b_q];
      d1 = t_data[b_q];
      d2 = 3.0 * d * tf;
      d = ((((((((delta1 * d - b_q0 * d) + a) - 4.0 * d1 * b_tmp * v0) +
               d2 * v0) -
              2.0 * d1 * b_tmp * v1) +
             d2 * v1) -
            delta1 * d1 * tf) +
           b_q0 * d1 * tf) /
          y_tmp;
      v_data[b_q] = d;
    }
  } else {
    binary_expand_op(v, delta1, b_q0, y_tmp, v0, t, b_tmp, tf, b_tmp, v1,
                     delta1, b_q0, y_tmp);
  }
  emxFree_real_T(&t);
}

/*
 * File trailer for planningTraj.c
 *
 * [EOF]
 */
