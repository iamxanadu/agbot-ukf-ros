/*
 * Academic License - for use in teaching, academic research, and meeting
 * course requirements at degree granting institutions only.  Not for
 * government, commercial, or other organizational use.
 *
 * agbot_simulinkblock_20191209.cpp
 *
 * Code generation for function 'agbot_simulinkblock_20191209'
 *
 */

/* Include files */
#include "agbot_simulinkblock_20191209.h"
#include "agbot_simulinkblock_20191209_data.h"
#include "agbot_simulinkblock_20191209_initialize.h"
#include "rt_nonfinite.h"
#include <cmath>
#include <math.h>

/* Function Declarations */
static double rt_powd_snf(double u0, double u1);

/* Function Definitions */
static double rt_powd_snf(double u0, double u1)
{
  double y;
  double d;
  double d1;
  if (rtIsNaN(u0) || rtIsNaN(u1)) {
    y = rtNaN;
  } else {
    d = std::abs(u0);
    d1 = std::abs(u1);
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
      y = std::sqrt(u0);
    } else if ((u0 < 0.0) && (u1 > std::floor(u1))) {
      y = rtNaN;
    } else {
      y = pow(u0, u1);
    }
  }

  return y;
}

void agbot_simulinkblock_20191209(double q_lmDt, double q_rmDt, const double
  VAR[16], double VARp[14], double *ax_imu, double *ay_imu, double *az_imu,
  double *wx_imu, double *wy_imu, double *wz_imu)
{
  double Z_idx_0;
  double Z_idx_7;
  double Z_idx_13_tmp;
  double Z_idx_5;
  double Z_idx_6;
  double Z_idx_10;
  double vrel5_Ny;
  double Z_idx_299;
  double Z_idx_102;
  double Z_idx_260;
  double Z_idx_3;
  double Z_idx_4;
  double Z_idx_1;
  double Z_idx_2;
  double Z_idx_157;
  double Z_idx_8;
  double Z_idx_152;
  double Z_idx_9;
  double Z_idx_11;
  double Z_idx_12;
  double Z_idx_69;
  double Z_idx_157_tmp;
  double b_Z_idx_157_tmp;
  double c_Z_idx_157_tmp;
  double Z_idx_166;
  double Z_idx_47;
  double Z_idx_44;
  double Z_idx_42;
  double Z_idx_469;
  double Z_idx_158_tmp;
  double Z_idx_165_tmp;
  double b_Z_idx_165_tmp;
  double c_Z_idx_165_tmp;
  double d_Z_idx_165_tmp;
  double Z_idx_165;
  double Z_idx_46;
  double Z_idx_45;
  double Z_idx_43;
  double Z_idx_356;
  double Z_idx_159_tmp;
  double b_Z_idx_159_tmp;
  double Z_idx_164_tmp;
  double b_Z_idx_164_tmp;
  double Z_idx_164;
  double Z_idx_171;
  double Z_idx_471;
  double Z_idx_317;
  double Z_idx_170_tmp;
  double Z_idx_170;
  double Z_idx_361;
  double Z_idx_305;
  double Z_idx_311;
  double Z_idx_169_tmp;
  double b_Z_idx_169_tmp;
  double Z_idx_169;
  double Z_idx_331;
  double Z_idx_472;
  double Z_idx_502;
  double Z_idx_145;
  double Z_idx_332_tmp;
  double vrel5_Tx;
  double Z_idx_464;
  double Z_idx_59;
  double Z_idx_141;
  double Z_idx_467;
  double Z_idx_332;
  double Z_idx_323;
  double vrel4_Ny;
  double Z_idx_333_tmp;
  double Z_idx_103;
  double Z_idx_160_tmp;
  double b_Z_idx_160_tmp;
  double c_Z_idx_160_tmp;
  double Z_idx_162_tmp;
  double b_Z_idx_162_tmp;
  double c_Z_idx_162_tmp;
  double Z_idx_163_tmp;
  double b_Z_idx_163_tmp;
  double c_Z_idx_163_tmp;
  double Z_idx_20;
  double Z_idx_18;
  double vrel1_Ny;
  double Fn8;
  double Z_idx_336;
  double vrel3_Ny;
  double Z_idx_287;
  double Z_idx_272;
  double vrel1_Tx;
  double vrel7_Nx;
  double Z_idx_113;
  double Z_idx_263;
  double Z_idx_58;
  double Z_idx_56;
  double Z_idx_181_tmp;
  double vrel5_Nx;
  double Z_idx_21;
  double Z_idx_505;
  double Z_idx_476;
  double vrel6_Nx;
  double Z_idx_338_tmp;
  double Z_idx_114;
  double Z_idx_183_tmp_tmp;
  double b_Z_idx_183_tmp_tmp;
  double Z_idx_183_tmp;
  double Z_idx_184_tmp;
  double b_Z_idx_184_tmp;
  double Z_idx_341;
  double Z_idx_120_tmp;
  double Z_idx_120;
  double Z_idx_266;
  double Z_idx_191_tmp;
  double b_Z_idx_191_tmp;
  double Z_idx_192_tmp;
  double Z_idx_19;
  double Z_idx_503;
  double Z_idx_343_tmp;
  double Z_idx_121;
  double Z_idx_193_tmp_tmp;
  double b_Z_idx_193_tmp_tmp;
  double Z_idx_193_tmp;
  double Z_idx_194_tmp;
  double b_Z_idx_194_tmp;
  double Z_idx_346;
  double Z_idx_127_tmp;
  double Z_idx_127;
  double Z_idx_201_tmp;
  double b_Z_idx_201_tmp;
  double Z_idx_202_tmp;
  double Z_idx_199_tmp;
  double Z_idx_17;
  double Z_idx_504;
  double Z_idx_348_tmp;
  double Z_idx_128;
  double Z_idx_203_tmp_tmp;
  double b_Z_idx_203_tmp_tmp;
  double Fn5;
  double Z_idx_204_tmp;
  double b_Z_idx_204_tmp;
  double Z_idx_351;
  double Fn1_tmp;
  double Fn1;
  double vrel1_Nx;
  double vrel2_Nx;
  double vrel3_Nx;
  double Fn4_tmp;
  double Fn4;
  double vrel4_Nx;
  double Z_idx_133;
  double Z_idx_215_tmp;
  double Z_idx_217_tmp;
  double b_Z_idx_217_tmp;
  double Z_idx_137;
  double Z_idx_275;
  double Z_idx_232_tmp;
  double Z_idx_278;
  if (isInitialized_agbot_simulinkblock_20191209 == false) {
    agbot_simulinkblock_20191209_initialize();
  }

  /* -------------------------------+--------------------------+-------------------+----------------- */
  /*  Quantity                      | Value                    | Units             | Description */
  /* -------------------------------|--------------------------|-------------------|----------------- */
  /*  m                   Constant */
  /*  N*s/m               Constant */
  /*  noUnits             Constant */
  /*  1/m                 Constant */
  /*  m/s                 Constant */
  /*  m/s^2               Constant */
  /*  m                   Constant */
  /*  kg*m^2              Constant */
  /*  kg*m^2              Constant */
  /*  kg*m^2              Constant */
  /*  s/m                 Constant */
  /*  s/rad               Constant */
  /*  1/N                 Constant */
  /*  N                   Constant */
  /*  m                   Constant */
  /*  noUnits             Constant */
  /*  m                   Constant */
  /*  m                   Constant */
  /*  m                   Constant */
  /*  m                   Constant */
  /*  m                   Constant */
  /*  kg                  Constant */
  /*  noUnits             Constant */
  /*  N*s/rad             Constant */
  /*  noUnits             Constant */
  /*  noUnits             Constant */
  /*  m                   Constant */
  /*  deg                 Constant */
  /*  deg                 Constant */
  /*  deg                 Initial Value */
  /*  deg                 Initial Value */
  /*  deg                 Initial Value */
  /*  deg                 Initial Value */
  /*  deg                 Initial Value */
  /*  m                   Initial Value */
  /*  m                   Initial Value */
  /*  m                   Initial Value */
  /*  rad/sec             Initial Value */
  /*  rad/sec             Initial Value */
  /*  rad/sec             Initial Value */
  /*  m/s                 Initial Value */
  /*  m/s                 Initial Value */
  /*  m/s                 Initial Value */
  /* -------------------------------+--------------------------+-------------------+----------------- */
  /*  Unit conversions */
  /*  Evaluate constants */
  /* =========================================================================== */
  /* =========================================================================== */
  /* =========================================================================== */
  /* =========================================================================== */
  Z_idx_0 = std::cos(VAR[0]);
  Z_idx_7 = std::cos(VAR[1]);
  Z_idx_13_tmp = Z_idx_0 * Z_idx_7;
  Z_idx_5 = std::sin(VAR[0]);
  Z_idx_6 = std::sin(VAR[1]);
  Z_idx_10 = Z_idx_0 * Z_idx_6;
  vrel5_Ny = 0.0945 * Z_idx_13_tmp + 0.0268;
  Z_idx_299 = vrel5_Ny + 0.14827 * Z_idx_5;
  Z_idx_102 = (Z_idx_299 - VAR[7]) - 0.18 * Z_idx_10;
  Z_idx_260 = 80.046 * (std::tanh(11613.5814 * (Z_idx_102 + 0.0446)) + 1.0) *
    rt_powd_snf(std::abs(Z_idx_102 + 0.0446), 0.4688);
  Z_idx_3 = std::sin(VAR[2]);
  Z_idx_4 = Z_idx_0 * Z_idx_3;
  Z_idx_1 = std::cos(VAR[2]);
  Z_idx_2 = Z_idx_0 * Z_idx_1;
  Z_idx_157 = Z_idx_1 * Z_idx_5;
  Z_idx_8 = Z_idx_157 * Z_idx_6 - Z_idx_3 * Z_idx_7;
  Z_idx_152 = Z_idx_3 * Z_idx_5;
  Z_idx_9 = Z_idx_1 * Z_idx_7 + Z_idx_152 * Z_idx_6;
  Z_idx_11 = Z_idx_3 * Z_idx_6 + Z_idx_157 * Z_idx_7;
  Z_idx_12 = Z_idx_152 * Z_idx_7 - Z_idx_1 * Z_idx_6;
  Z_idx_1 = (Z_idx_2 * Z_idx_4 + Z_idx_8 * Z_idx_9) + Z_idx_11 * Z_idx_12;
  Z_idx_69 = Z_idx_4 * Z_idx_4;
  Z_idx_157_tmp = Z_idx_4 * Z_idx_5;
  b_Z_idx_157_tmp = Z_idx_9 * Z_idx_10;
  c_Z_idx_157_tmp = Z_idx_12 * Z_idx_13_tmp;
  Z_idx_157 = (b_Z_idx_157_tmp + c_Z_idx_157_tmp) - Z_idx_157_tmp;
  Z_idx_166 = 0.0268 * Z_idx_2;
  Z_idx_47 = -0.0945 * Z_idx_10 - 0.18 * Z_idx_13_tmp;
  Z_idx_44 = 0.14827 * Z_idx_13_tmp - 0.0945 * Z_idx_5;
  Z_idx_42 = -0.14827 * Z_idx_10 - 0.18 * Z_idx_5;
  Z_idx_469 = Z_idx_4 * Z_idx_47;
  Z_idx_158_tmp = Z_idx_9 * Z_idx_44;
  Z_idx_165_tmp = Z_idx_8 * Z_idx_10;
  b_Z_idx_165_tmp = Z_idx_11 * Z_idx_13_tmp;
  c_Z_idx_165_tmp = Z_idx_2 * Z_idx_5;
  d_Z_idx_165_tmp = (c_Z_idx_165_tmp - Z_idx_165_tmp) - b_Z_idx_165_tmp;
  Z_idx_165 = 0.0268 * d_Z_idx_165_tmp;
  Z_idx_46 = 0.18 * Z_idx_6 - 0.0945 * Z_idx_7;
  Z_idx_45 = 0.14827 * Z_idx_6;
  Z_idx_43 = 0.14827 * Z_idx_7;
  Z_idx_356 = Z_idx_4 * Z_idx_46;
  Z_idx_159_tmp = Z_idx_9 * Z_idx_45;
  b_Z_idx_159_tmp = Z_idx_12 * Z_idx_43;
  Z_idx_164_tmp = Z_idx_7 * Z_idx_8;
  b_Z_idx_164_tmp = Z_idx_6 * Z_idx_11;
  Z_idx_164 = 0.0268 * (b_Z_idx_164_tmp - Z_idx_164_tmp);
  Z_idx_3 = Z_idx_2 * Z_idx_2;
  Z_idx_152 = (Z_idx_165_tmp + b_Z_idx_165_tmp) - c_Z_idx_165_tmp;
  Z_idx_171 = 0.0268 * Z_idx_4;
  Z_idx_471 = Z_idx_2 * Z_idx_47;
  Z_idx_317 = Z_idx_8 * Z_idx_44;
  Z_idx_170_tmp = (Z_idx_157_tmp - b_Z_idx_157_tmp) - c_Z_idx_157_tmp;
  Z_idx_170 = 0.0268 * Z_idx_170_tmp;
  Z_idx_361 = Z_idx_2 * Z_idx_46;
  Z_idx_305 = Z_idx_8 * Z_idx_45;
  Z_idx_311 = Z_idx_11 * Z_idx_43;
  Z_idx_169_tmp = Z_idx_7 * Z_idx_9;
  b_Z_idx_169_tmp = Z_idx_6 * Z_idx_12;
  Z_idx_169 = 0.0268 * (b_Z_idx_169_tmp - Z_idx_169_tmp);
  Z_idx_331 = std::sqrt(Z_idx_3 + Z_idx_69);
  Z_idx_472 = 0.029003951599999998 * Z_idx_9 * q_lmDt;
  Z_idx_502 = ((Z_idx_3 + Z_idx_8 * Z_idx_8) + Z_idx_11 * Z_idx_11) * VAR[11];
  Z_idx_145 = Z_idx_1 * VAR[12];
  Z_idx_332_tmp = Z_idx_152 * VAR[13];
  vrel5_Tx = ((0.0945 * Z_idx_8 + 0.18 * Z_idx_11) - Z_idx_171) * VAR[9];
  Z_idx_464 = (((Z_idx_471 + Z_idx_317) + Z_idx_11 * Z_idx_42) + Z_idx_170) *
    VAR[10];
  Z_idx_59 = (((Z_idx_361 - Z_idx_305) - Z_idx_311) + Z_idx_169) * VAR[8];
  Z_idx_141 = (Z_idx_502 + Z_idx_145) + Z_idx_332_tmp;
  Z_idx_467 = Z_idx_141 + vrel5_Tx;
  Z_idx_332 = ((Z_idx_467 + Z_idx_464) + Z_idx_59) - Z_idx_472;
  Z_idx_323 = (Z_idx_1 * VAR[11] + ((Z_idx_69 + Z_idx_9 * Z_idx_9) + Z_idx_12 *
    Z_idx_12) * VAR[12]) + Z_idx_157 * VAR[13];
  vrel4_Ny = (Z_idx_323 + 0.029003951599999998 * Z_idx_8 * q_lmDt) + ((0.0945 *
    Z_idx_9 + 0.18 * Z_idx_12) + Z_idx_166) * VAR[9];
  Z_idx_333_tmp = (vrel4_Ny + (((Z_idx_469 + Z_idx_158_tmp) + Z_idx_12 *
    Z_idx_42) - Z_idx_165) * VAR[10]) + (((Z_idx_356 - Z_idx_159_tmp) -
    b_Z_idx_159_tmp) - Z_idx_164) * VAR[8];
  Z_idx_103 = 0.14827 * Z_idx_0;
  Z_idx_160_tmp = Z_idx_5 * Z_idx_46;
  b_Z_idx_160_tmp = Z_idx_10 * Z_idx_45;
  c_Z_idx_160_tmp = Z_idx_13_tmp * Z_idx_43;
  Z_idx_162_tmp = Z_idx_5 * Z_idx_5;
  b_Z_idx_162_tmp = Z_idx_10 * Z_idx_10;
  c_Z_idx_162_tmp = Z_idx_13_tmp * Z_idx_13_tmp;
  Z_idx_163_tmp = Z_idx_5 * Z_idx_47;
  b_Z_idx_163_tmp = Z_idx_10 * Z_idx_44;
  c_Z_idx_163_tmp = Z_idx_13_tmp * Z_idx_42;
  Z_idx_20 = -Z_idx_0 * Z_idx_6 * VAR[9] - Z_idx_5 * Z_idx_7 * VAR[8];
  Z_idx_18 = Z_idx_13_tmp * VAR[9] - Z_idx_5 * Z_idx_6 * VAR[8];
  vrel1_Ny = Z_idx_152 * VAR[11] + Z_idx_157 * VAR[12];
  Z_idx_157 = (0.0945 * Z_idx_10 + 0.18 * Z_idx_13_tmp) * VAR[9];
  Fn8 = (((Z_idx_162_tmp + -1.0) + b_Z_idx_162_tmp) + c_Z_idx_162_tmp) * VAR[13];
  Z_idx_3 = (((((vrel1_Ny + (((Z_idx_103 - Z_idx_160_tmp) - b_Z_idx_160_tmp) -
    c_Z_idx_160_tmp) * VAR[8]) + Z_idx_157) + Fn8) + ((b_Z_idx_163_tmp +
    c_Z_idx_163_tmp) - Z_idx_163_tmp) * VAR[10]) + 0.0945 * Z_idx_20) - 0.18 *
    Z_idx_18;
  Z_idx_336 = std::sqrt((Z_idx_332 * Z_idx_332 + Z_idx_333_tmp * Z_idx_333_tmp)
                        + Z_idx_3 * Z_idx_3);
  vrel3_Ny = (((Z_idx_472 - Z_idx_502) - Z_idx_145) - Z_idx_332_tmp) - vrel5_Tx;
  Z_idx_287 = (vrel3_Ny - Z_idx_464) - Z_idx_59;
  Z_idx_272 = Z_idx_4 * Z_idx_333_tmp - Z_idx_2 * Z_idx_287;
  vrel1_Tx = Z_idx_272 / (Z_idx_331 * (Z_idx_336 + 0.01));
  vrel7_Nx = vrel5_Ny + 0.05760333333333334 * Z_idx_5;
  Z_idx_113 = (vrel7_Nx - VAR[7]) - 0.18 * Z_idx_10;
  Z_idx_263 = 80.046 * (std::tanh(11613.5814 * (Z_idx_113 + 0.0446)) + 1.0) *
    rt_powd_snf(std::abs(Z_idx_113 + 0.0446), 0.4688);
  Z_idx_58 = 0.090666666666666673 * Z_idx_13_tmp;
  Z_idx_59 = Z_idx_44 - Z_idx_58;
  Z_idx_56 = 0.090666666666666673 * Z_idx_10;
  Z_idx_152 = Z_idx_42 + Z_idx_56;
  Z_idx_181_tmp = Z_idx_9 * Z_idx_59;
  Z_idx_332 = 0.090666666666666673 * Z_idx_6;
  Z_idx_46 = Z_idx_332 - Z_idx_45;
  Z_idx_44 = 0.090666666666666673 * Z_idx_7;
  Z_idx_47 = Z_idx_44 - Z_idx_43;
  vrel5_Nx = Z_idx_9 * Z_idx_46;
  Z_idx_21 = Z_idx_12 * Z_idx_47;
  Z_idx_505 = Z_idx_8 * Z_idx_46;
  Z_idx_476 = Z_idx_11 * Z_idx_47;
  vrel6_Nx = Z_idx_8 * Z_idx_59;
  vrel5_Tx = (Z_idx_169 + ((Z_idx_361 + Z_idx_505) + Z_idx_476)) * VAR[8];
  Z_idx_464 = (Z_idx_170 + ((Z_idx_471 + vrel6_Nx) + Z_idx_11 * Z_idx_152)) *
    VAR[10];
  Z_idx_1 = ((Z_idx_467 + vrel5_Tx) + Z_idx_464) - Z_idx_472;
  Z_idx_338_tmp = (vrel4_Ny + (((Z_idx_469 + Z_idx_181_tmp) + Z_idx_12 *
    Z_idx_152) - Z_idx_165) * VAR[10]) + (((Z_idx_356 + vrel5_Nx) + Z_idx_21) -
    Z_idx_164) * VAR[8];
  Z_idx_114 = 0.05760333333333334 * Z_idx_0;
  Z_idx_183_tmp_tmp = Z_idx_10 * Z_idx_46;
  b_Z_idx_183_tmp_tmp = Z_idx_13_tmp * Z_idx_47;
  Z_idx_183_tmp = (Z_idx_114 + Z_idx_183_tmp_tmp) + b_Z_idx_183_tmp_tmp;
  Z_idx_184_tmp = Z_idx_10 * Z_idx_59;
  b_Z_idx_184_tmp = Z_idx_13_tmp * Z_idx_152;
  Z_idx_43 = (vrel1_Ny + Z_idx_157) + Fn8;
  Z_idx_3 = (((Z_idx_43 + (Z_idx_183_tmp - Z_idx_160_tmp) * VAR[8]) +
              ((Z_idx_184_tmp + b_Z_idx_184_tmp) - Z_idx_163_tmp) * VAR[10]) +
             0.0945 * Z_idx_20) - 0.18 * Z_idx_18;
  Z_idx_341 = std::sqrt((Z_idx_1 * Z_idx_1 + Z_idx_338_tmp * Z_idx_338_tmp) +
                        Z_idx_3 * Z_idx_3);
  vrel5_Tx = (vrel3_Ny - vrel5_Tx) - Z_idx_464;
  Z_idx_120_tmp = vrel5_Ny + -0.033063333333333333 * Z_idx_5;
  Z_idx_120 = (Z_idx_120_tmp - VAR[7]) - 0.18 * Z_idx_10;
  Z_idx_266 = 80.046 * (std::tanh(11613.5814 * (Z_idx_120 + 0.0446)) + 1.0) *
    rt_powd_snf(std::abs(Z_idx_120 + 0.0446), 0.4688);
  Z_idx_157 = Z_idx_332 + Z_idx_46;
  Z_idx_47 += Z_idx_44;
  Z_idx_191_tmp = Z_idx_9 * Z_idx_157;
  b_Z_idx_191_tmp = Z_idx_12 * Z_idx_47;
  Z_idx_69 = Z_idx_59 - Z_idx_58;
  Z_idx_152 += Z_idx_56;
  Z_idx_192_tmp = Z_idx_9 * Z_idx_69;
  Z_idx_59 = Z_idx_8 * Z_idx_157;
  Z_idx_19 = Z_idx_11 * Z_idx_47;
  Z_idx_503 = Z_idx_8 * Z_idx_69;
  Z_idx_464 = (Z_idx_169 + ((Z_idx_361 + Z_idx_59) + Z_idx_19)) * VAR[8];
  Z_idx_46 = (Z_idx_170 + ((Z_idx_471 + Z_idx_503) + Z_idx_11 * Z_idx_152)) *
    VAR[10];
  Z_idx_3 = ((Z_idx_467 + Z_idx_464) + Z_idx_46) - Z_idx_472;
  Z_idx_343_tmp = (vrel4_Ny + (((Z_idx_356 + Z_idx_191_tmp) + b_Z_idx_191_tmp) -
    Z_idx_164) * VAR[8]) + (((Z_idx_469 + Z_idx_192_tmp) + Z_idx_12 * Z_idx_152)
    - Z_idx_165) * VAR[10];
  Z_idx_121 = -0.033063333333333333 * Z_idx_0;
  Z_idx_193_tmp_tmp = Z_idx_10 * Z_idx_157;
  b_Z_idx_193_tmp_tmp = Z_idx_13_tmp * Z_idx_47;
  Z_idx_193_tmp = (Z_idx_121 + Z_idx_193_tmp_tmp) + b_Z_idx_193_tmp_tmp;
  Z_idx_194_tmp = Z_idx_10 * Z_idx_69;
  b_Z_idx_194_tmp = Z_idx_13_tmp * Z_idx_152;
  Z_idx_1 = (((Z_idx_43 + (Z_idx_193_tmp - Z_idx_160_tmp) * VAR[8]) +
              ((Z_idx_194_tmp + b_Z_idx_194_tmp) - Z_idx_163_tmp) * VAR[10]) +
             0.0945 * Z_idx_20) - 0.18 * Z_idx_18;
  Z_idx_346 = std::sqrt((Z_idx_3 * Z_idx_3 + Z_idx_343_tmp * Z_idx_343_tmp) +
                        Z_idx_1 * Z_idx_1);
  Z_idx_42 = (vrel3_Ny - Z_idx_464) - Z_idx_46;
  Z_idx_127_tmp = vrel5_Ny + -0.12373 * Z_idx_5;
  Z_idx_127 = (Z_idx_127_tmp - VAR[7]) - 0.18 * Z_idx_10;
  Z_idx_45 = 80.046 * (std::tanh(11613.5814 * (Z_idx_127 + 0.0446)) + 1.0) *
    rt_powd_snf(std::abs(Z_idx_127 + 0.0446), 0.4688);
  Z_idx_157 += Z_idx_332;
  Z_idx_3 = Z_idx_44 + Z_idx_47;
  Z_idx_201_tmp = Z_idx_9 * Z_idx_157;
  b_Z_idx_201_tmp = Z_idx_12 * Z_idx_3;
  Z_idx_46 = Z_idx_69 - Z_idx_58;
  Z_idx_47 = Z_idx_56 + Z_idx_152;
  Z_idx_202_tmp = Z_idx_9 * Z_idx_46;
  Z_idx_199_tmp = Z_idx_8 * Z_idx_157;
  Z_idx_17 = Z_idx_11 * Z_idx_3;
  Z_idx_504 = Z_idx_8 * Z_idx_46;
  Z_idx_332 = (Z_idx_169 + ((Z_idx_361 + Z_idx_199_tmp) + Z_idx_17)) * VAR[8];
  Z_idx_152 = (Z_idx_170 + ((Z_idx_471 + Z_idx_504) + Z_idx_11 * Z_idx_47)) *
    VAR[10];
  Z_idx_1 = ((Z_idx_467 + Z_idx_332) + Z_idx_152) - Z_idx_472;
  Z_idx_348_tmp = (vrel4_Ny + (((Z_idx_356 + Z_idx_201_tmp) + b_Z_idx_201_tmp) -
    Z_idx_164) * VAR[8]) + (((Z_idx_469 + Z_idx_202_tmp) + Z_idx_12 * Z_idx_47)
    - Z_idx_165) * VAR[10];
  Z_idx_128 = -0.12373 * Z_idx_0;
  Z_idx_203_tmp_tmp = Z_idx_10 * Z_idx_157;
  b_Z_idx_203_tmp_tmp = Z_idx_13_tmp * Z_idx_3;
  Fn5 = (Z_idx_128 + Z_idx_203_tmp_tmp) + b_Z_idx_203_tmp_tmp;
  Z_idx_204_tmp = Z_idx_10 * Z_idx_46;
  b_Z_idx_204_tmp = Z_idx_13_tmp * Z_idx_47;
  Z_idx_3 = (((Z_idx_43 + (Fn5 - Z_idx_160_tmp) * VAR[8]) + ((Z_idx_204_tmp +
    b_Z_idx_204_tmp) - Z_idx_163_tmp) * VAR[10]) + 0.0945 * Z_idx_20) - 0.18 *
    Z_idx_18;
  Z_idx_351 = std::sqrt((Z_idx_1 * Z_idx_1 + Z_idx_348_tmp * Z_idx_348_tmp) +
                        Z_idx_3 * Z_idx_3);
  Z_idx_47 = (vrel3_Ny - Z_idx_332) - Z_idx_152;
  Z_idx_3 = Z_idx_4 * Z_idx_348_tmp - Z_idx_2 * Z_idx_47;
  Z_idx_1 = Z_idx_3 / (Z_idx_331 * (Z_idx_351 + 0.01));
  Z_idx_46 = ((Z_idx_260 * vrel1_Tx + Z_idx_263 * ((Z_idx_4 * Z_idx_338_tmp -
    Z_idx_2 * vrel5_Tx) / (Z_idx_331 * (Z_idx_341 + 0.01)))) + Z_idx_266 *
              ((Z_idx_4 * Z_idx_343_tmp - Z_idx_2 * Z_idx_42) / (Z_idx_331 *
    (Z_idx_346 + 0.01)))) + Z_idx_45 * Z_idx_1;
  Fn1_tmp = Z_idx_260 - (0.58778525229247325 * (0.5 * std::tanh(30.0 * Z_idx_272
    / Z_idx_331 + -3.0) + 0.5) * Z_idx_46 / (1.0 - 0.58778525229247325 *
    vrel1_Tx) - 0.25 * q_lmDt * (1.0 - std::tanh(30.0 * q_lmDt + 3.0)));
  Fn1 = 0.5 * Fn1_tmp * (std::tanh(30.0 * Fn1_tmp + -3.0) + 1.0);
  vrel1_Nx = -Z_idx_287 / (Z_idx_336 + 0.01);
  vrel2_Nx = -vrel5_Tx / (Z_idx_341 + 0.01);
  vrel3_Nx = -Z_idx_42 / (Z_idx_346 + 0.01);
  Fn4_tmp = Z_idx_45 - (0.25 * q_lmDt * (std::tanh(30.0 * q_lmDt + -3.0) + 1.0)
                        - 0.79863551004729294 * (0.5 - 0.5 * std::tanh(30.0 *
    Z_idx_3 / Z_idx_331 + 3.0)) * Z_idx_46 / (0.79863551004729294 * Z_idx_1 +
    1.0));
  Fn4 = 0.5 * Fn4_tmp * (std::tanh(30.0 * Fn4_tmp + -3.0) + 1.0);
  vrel4_Nx = -Z_idx_47 / (Z_idx_351 + 0.01);
  Z_idx_133 = (Z_idx_299 + 0.18 * Z_idx_10) - VAR[7];
  Z_idx_272 = 80.046 * (std::tanh(11613.5814 * (Z_idx_133 + 0.0446)) + 1.0) *
    rt_powd_snf(std::abs(Z_idx_133 + 0.0446), 0.4688);
  Z_idx_1 = 0.18 * Z_idx_13_tmp - 0.0945 * Z_idx_10;
  Z_idx_42 = 0.18 * Z_idx_5 - 0.14827 * Z_idx_10;
  vrel3_Ny = Z_idx_4 * Z_idx_1;
  Z_idx_3 = -0.0945 * Z_idx_7 - 0.18 * Z_idx_6;
  Z_idx_469 = Z_idx_4 * Z_idx_3;
  Z_idx_464 = Z_idx_2 * Z_idx_3;
  Z_idx_467 = Z_idx_2 * Z_idx_1;
  Z_idx_472 = 0.028830707876370893 * Z_idx_9 * q_rmDt;
  Z_idx_152 = (Z_idx_169 + ((Z_idx_464 - Z_idx_305) - Z_idx_311)) * VAR[8];
  Z_idx_47 = (Z_idx_170 + ((Z_idx_467 + Z_idx_317) + Z_idx_11 * Z_idx_42)) *
    VAR[10];
  Z_idx_43 = ((0.0945 * Z_idx_8 - 0.18 * Z_idx_11) - Z_idx_171) * VAR[9];
  Z_idx_46 = (((Z_idx_141 + Z_idx_152) + Z_idx_47) + Z_idx_43) - Z_idx_472;
  vrel4_Ny = (Z_idx_323 + 0.028830707876370893 * Z_idx_8 * q_rmDt) + (Z_idx_166
    + (0.0945 * Z_idx_9 - 0.18 * Z_idx_12)) * VAR[9];
  Z_idx_287 = (vrel4_Ny + (((vrel3_Ny + Z_idx_158_tmp) + Z_idx_12 * Z_idx_42) -
    Z_idx_165) * VAR[10]) + (((Z_idx_469 - Z_idx_159_tmp) - b_Z_idx_159_tmp) -
    Z_idx_164) * VAR[8];
  Z_idx_215_tmp = Z_idx_5 * Z_idx_3;
  Z_idx_217_tmp = Z_idx_5 * Z_idx_1;
  b_Z_idx_217_tmp = Z_idx_13_tmp * Z_idx_42;
  Z_idx_332 = vrel1_Ny + Fn8;
  Z_idx_44 = (0.0945 * Z_idx_10 - 0.18 * Z_idx_13_tmp) * VAR[9];
  Z_idx_3 = ((((Z_idx_332 + (((Z_idx_103 - Z_idx_215_tmp) - b_Z_idx_160_tmp) -
    c_Z_idx_160_tmp) * VAR[8]) + Z_idx_44) + ((b_Z_idx_163_tmp + b_Z_idx_217_tmp)
    - Z_idx_217_tmp) * VAR[10]) + 0.0945 * Z_idx_20) + 0.18 * Z_idx_18;
  Z_idx_356 = std::sqrt((Z_idx_46 * Z_idx_46 + Z_idx_287 * Z_idx_287) + Z_idx_3 *
                        Z_idx_3);
  Z_idx_157 = ((Z_idx_472 - Z_idx_502) - Z_idx_145) - Z_idx_332_tmp;
  vrel5_Ny = ((Z_idx_157 - Z_idx_152) - Z_idx_47) - Z_idx_43;
  Z_idx_58 = Z_idx_4 * Z_idx_287 - Z_idx_2 * vrel5_Ny;
  vrel5_Tx = Z_idx_58 / (Z_idx_331 * (Z_idx_356 + 0.01));
  Z_idx_137 = (vrel7_Nx + 0.18 * Z_idx_10) - VAR[7];
  Z_idx_275 = 80.046 * (std::tanh(11613.5814 * (Z_idx_137 + 0.0446)) + 1.0) *
    rt_powd_snf(std::abs(Z_idx_137 + 0.0446), 0.4688);
  Z_idx_152 = Z_idx_56 + Z_idx_42;
  Z_idx_47 = (Z_idx_169 + ((Z_idx_464 + Z_idx_505) + Z_idx_476)) * VAR[8];
  Z_idx_46 = (Z_idx_170 + ((Z_idx_467 + vrel6_Nx) + Z_idx_11 * Z_idx_152)) *
    VAR[10];
  Z_idx_45 = Z_idx_141 + Z_idx_43;
  Z_idx_1 = ((Z_idx_45 + Z_idx_47) + Z_idx_46) - Z_idx_472;
  Z_idx_471 = (vrel4_Ny + (((vrel3_Ny + Z_idx_181_tmp) + Z_idx_12 * Z_idx_152) -
    Z_idx_165) * VAR[10]) + (((Z_idx_469 + vrel5_Nx) + Z_idx_21) - Z_idx_164) *
    VAR[8];
  Z_idx_232_tmp = Z_idx_13_tmp * Z_idx_152;
  Z_idx_42 = Z_idx_332 + Z_idx_44;
  Z_idx_3 = (((Z_idx_42 + (Z_idx_183_tmp - Z_idx_215_tmp) * VAR[8]) +
              ((Z_idx_184_tmp + Z_idx_232_tmp) - Z_idx_217_tmp) * VAR[10]) +
             0.0945 * Z_idx_20) + 0.18 * Z_idx_18;
  Z_idx_361 = std::sqrt((Z_idx_1 * Z_idx_1 + Z_idx_471 * Z_idx_471) + Z_idx_3 *
                        Z_idx_3);
  Z_idx_44 = Z_idx_157 - Z_idx_43;
  Z_idx_43 = (Z_idx_44 - Z_idx_47) - Z_idx_46;
  Z_idx_141 = (Z_idx_120_tmp + 0.18 * Z_idx_10) - VAR[7];
  Z_idx_278 = 80.046 * (std::tanh(11613.5814 * (Z_idx_141 + 0.0446)) + 1.0) *
    rt_powd_snf(std::abs(Z_idx_141 + 0.0446), 0.4688);
  Z_idx_157 = Z_idx_56 + Z_idx_152;
  Z_idx_47 = (Z_idx_169 + ((Z_idx_464 + Z_idx_59) + Z_idx_19)) * VAR[8];
  Z_idx_46 = (Z_idx_170 + ((Z_idx_467 + Z_idx_503) + Z_idx_11 * Z_idx_157)) *
    VAR[10];
  Z_idx_1 = ((Z_idx_45 + Z_idx_47) + Z_idx_46) - Z_idx_472;
  Z_idx_69 = (vrel4_Ny + (((Z_idx_469 + Z_idx_191_tmp) + b_Z_idx_191_tmp) -
    Z_idx_164) * VAR[8]) + (((vrel3_Ny + Z_idx_192_tmp) + Z_idx_12 * Z_idx_157)
    - Z_idx_165) * VAR[10];
  Z_idx_332_tmp = Z_idx_13_tmp * Z_idx_157;
  Z_idx_3 = (((Z_idx_42 + (Z_idx_193_tmp - Z_idx_215_tmp) * VAR[8]) +
              ((Z_idx_194_tmp + Z_idx_332_tmp) - Z_idx_217_tmp) * VAR[10]) +
             0.0945 * Z_idx_20) + 0.18 * Z_idx_18;
  Z_idx_59 = std::sqrt((Z_idx_1 * Z_idx_1 + Z_idx_69 * Z_idx_69) + Z_idx_3 *
                       Z_idx_3);
  Z_idx_152 = (Z_idx_44 - Z_idx_47) - Z_idx_46;
  Z_idx_145 = (Z_idx_127_tmp + 0.18 * Z_idx_10) - VAR[7];
  Z_idx_332 = 80.046 * (std::tanh(11613.5814 * (Z_idx_145 + 0.0446)) + 1.0) *
    rt_powd_snf(std::abs(Z_idx_145 + 0.0446), 0.4688);
  Z_idx_3 = Z_idx_56 + Z_idx_157;
  Z_idx_46 = (Z_idx_169 + ((Z_idx_464 + Z_idx_199_tmp) + Z_idx_17)) * VAR[8];
  Z_idx_464 = (Z_idx_170 + ((Z_idx_467 + Z_idx_504) + Z_idx_11 * Z_idx_3)) *
    VAR[10];
  Z_idx_1 = ((Z_idx_45 + Z_idx_46) + Z_idx_464) - Z_idx_472;
  Z_idx_157 = (vrel4_Ny + (((Z_idx_469 + Z_idx_201_tmp) + b_Z_idx_201_tmp) -
    Z_idx_164) * VAR[8]) + (((vrel3_Ny + Z_idx_202_tmp) + Z_idx_12 * Z_idx_3) -
    Z_idx_165) * VAR[10];
  Z_idx_183_tmp = Z_idx_13_tmp * Z_idx_3;
  Z_idx_3 = (((Z_idx_42 + (Fn5 - Z_idx_215_tmp) * VAR[8]) + ((Z_idx_204_tmp +
    Z_idx_183_tmp) - Z_idx_217_tmp) * VAR[10]) + 0.0945 * Z_idx_20) + 0.18 *
    Z_idx_18;
  Z_idx_47 = std::sqrt((Z_idx_1 * Z_idx_1 + Z_idx_157 * Z_idx_157) + Z_idx_3 *
                       Z_idx_3);
  Z_idx_46 = (Z_idx_44 - Z_idx_46) - Z_idx_464;
  Z_idx_3 = Z_idx_4 * Z_idx_157 - Z_idx_2 * Z_idx_46;
  Z_idx_1 = Z_idx_3 / (Z_idx_331 * (Z_idx_47 + 0.01));
  Z_idx_464 = ((Z_idx_272 * vrel5_Tx + Z_idx_275 * ((Z_idx_4 * Z_idx_471 -
    Z_idx_2 * Z_idx_43) / (Z_idx_331 * (Z_idx_361 + 0.01)))) + Z_idx_278 *
               ((Z_idx_4 * Z_idx_69 - Z_idx_2 * Z_idx_152) / (Z_idx_331 *
    (Z_idx_59 + 0.01)))) + Z_idx_332 * Z_idx_1;
  Z_idx_202_tmp = Z_idx_272 - (0.58778525229247325 * (0.5 * std::tanh(30.0 *
    Z_idx_58 / Z_idx_331 + -3.0) + 0.5) * Z_idx_464 / (1.0 - 0.58778525229247325
    * vrel5_Tx) - 0.25 * q_rmDt * (1.0 - std::tanh(30.0 * q_rmDt + 3.0)));
  Fn5 = 0.5 * Z_idx_202_tmp * (std::tanh(30.0 * Z_idx_202_tmp + -3.0) + 1.0);
  vrel5_Nx = -vrel5_Ny / (Z_idx_356 + 0.01);
  vrel6_Nx = -Z_idx_43 / (Z_idx_361 + 0.01);
  vrel7_Nx = -Z_idx_152 / (Z_idx_59 + 0.01);
  Z_idx_199_tmp = Z_idx_332 - (0.25 * q_rmDt * (std::tanh(30.0 * q_rmDt + -3.0)
    + 1.0) - 0.79863551004729294 * (0.5 - 0.5 * std::tanh(30.0 * Z_idx_3 /
    Z_idx_331 + 3.0)) * Z_idx_464 / (0.79863551004729294 * Z_idx_1 + 1.0));
  Fn8 = 0.5 * Z_idx_199_tmp * (std::tanh(30.0 * Z_idx_199_tmp + -3.0) + 1.0);
  Z_idx_166 = -Z_idx_46 / (Z_idx_47 + 0.01);
  Z_idx_502 = (((((((((((((((-(Fn1 * vrel1_Nx) - Z_idx_263 * vrel2_Nx) -
    Z_idx_266 * vrel3_Nx) - Fn4 * vrel4_Nx) - Fn5 * vrel5_Nx) - Z_idx_275 *
    vrel6_Nx) - Z_idx_278 * vrel7_Nx) - Fn8 * Z_idx_166) - Z_idx_263 *
                      d_Z_idx_165_tmp) - Z_idx_266 * d_Z_idx_165_tmp) -
                    Z_idx_275 * d_Z_idx_165_tmp) - Z_idx_278 * d_Z_idx_165_tmp)
                  - Fn1_tmp * d_Z_idx_165_tmp) - Fn4_tmp * d_Z_idx_165_tmp) -
                Z_idx_202_tmp * d_Z_idx_165_tmp) - Z_idx_199_tmp *
               d_Z_idx_165_tmp) / 14.0;
  vrel1_Ny = Z_idx_333_tmp / (Z_idx_336 + 0.01);
  Z_idx_272 = Z_idx_338_tmp / (Z_idx_341 + 0.01);
  vrel3_Ny = Z_idx_343_tmp / (Z_idx_346 + 0.01);
  vrel4_Ny = Z_idx_348_tmp / (Z_idx_351 + 0.01);
  vrel5_Ny = Z_idx_287 / (Z_idx_356 + 0.01);
  vrel5_Tx = Z_idx_471 / (Z_idx_361 + 0.01);
  Z_idx_45 = Z_idx_69 / (Z_idx_59 + 0.01);
  Z_idx_42 = Z_idx_157 / (Z_idx_47 + 0.01);
  Z_idx_503 = (((((((((((((((-(Fn1 * vrel1_Ny) - Z_idx_263 * Z_idx_272) -
    Z_idx_266 * vrel3_Ny) - Fn4 * vrel4_Ny) - Fn5 * vrel5_Ny) - Z_idx_275 *
    vrel5_Tx) - Z_idx_278 * Z_idx_45) - Fn8 * Z_idx_42) - Z_idx_263 *
                      Z_idx_170_tmp) - Z_idx_266 * Z_idx_170_tmp) - Z_idx_275 *
                    Z_idx_170_tmp) - Z_idx_278 * Z_idx_170_tmp) - Fn1_tmp *
                  Z_idx_170_tmp) - Fn4_tmp * Z_idx_170_tmp) - Z_idx_202_tmp *
                Z_idx_170_tmp) - Z_idx_199_tmp * Z_idx_170_tmp) / 14.0;
  Z_idx_287 = 25.0 * Z_idx_102 * (((Z_idx_102 * Z_idx_103 * VAR[8] + 0.0945 *
    Z_idx_102 * Z_idx_20) - Z_idx_102 * VAR[13]) - 0.5 * (0.36 * Z_idx_102) *
    Z_idx_18) / (Z_idx_102 * Z_idx_102);
  Z_idx_260 = 25.0 * Z_idx_113 * (((Z_idx_113 * Z_idx_114 * VAR[8] + 0.0945 *
    Z_idx_113 * Z_idx_20) - Z_idx_113 * VAR[13]) - 0.5 * (0.36 * Z_idx_113) *
    Z_idx_18) / (Z_idx_113 * Z_idx_113);
  Z_idx_299 = 25.0 * Z_idx_120 * (((Z_idx_120 * Z_idx_121 * VAR[8] + 0.0945 *
    Z_idx_120 * Z_idx_20) - Z_idx_120 * VAR[13]) - 0.5 * (0.36 * Z_idx_120) *
    Z_idx_18) / (Z_idx_120 * Z_idx_120);
  Z_idx_305 = 25.0 * Z_idx_127 * (((Z_idx_127 * Z_idx_128 * VAR[8] + 0.0945 *
    Z_idx_127 * Z_idx_20) - Z_idx_127 * VAR[13]) - 0.5 * (0.36 * Z_idx_127) *
    Z_idx_18) / (Z_idx_127 * Z_idx_127);
  Z_idx_311 = 25.0 * Z_idx_133 * (((Z_idx_103 * Z_idx_133 * VAR[8] + 0.0945 *
    Z_idx_133 * Z_idx_20) + 0.5 * (0.36 * Z_idx_133) * Z_idx_18) - Z_idx_133 *
    VAR[13]) / (Z_idx_133 * Z_idx_133);
  Z_idx_317 = 25.0 * Z_idx_137 * (((Z_idx_114 * Z_idx_137 * VAR[8] + 0.0945 *
    Z_idx_137 * Z_idx_20) + 0.5 * (0.36 * Z_idx_137) * Z_idx_18) - Z_idx_137 *
    VAR[13]) / (Z_idx_137 * Z_idx_137);
  Z_idx_323 = 25.0 * Z_idx_141 * (((Z_idx_121 * Z_idx_141 * VAR[8] + 0.0945 *
    Z_idx_141 * Z_idx_20) + 0.5 * (0.36 * Z_idx_141) * Z_idx_18) - Z_idx_141 *
    VAR[13]) / (Z_idx_141 * Z_idx_141);
  Z_idx_171 = 25.0 * Z_idx_145 * (((Z_idx_128 * Z_idx_145 * VAR[8] + 0.0945 *
    Z_idx_145 * Z_idx_20) + 0.5 * (0.36 * Z_idx_145) * Z_idx_18) - Z_idx_145 *
    VAR[13]) / (Z_idx_145 * Z_idx_145);
  Z_idx_3 = (Z_idx_162_tmp + b_Z_idx_162_tmp) + c_Z_idx_162_tmp;
  Z_idx_504 = ((((((((((((((((Z_idx_263 * Z_idx_3 + Z_idx_266 * Z_idx_3) +
    Z_idx_275 * Z_idx_3) + Z_idx_278 * Z_idx_3) + Fn1_tmp * Z_idx_3) + Fn4_tmp *
    Z_idx_3) + Z_idx_202_tmp * Z_idx_3) + Z_idx_199_tmp * Z_idx_3) + Z_idx_287)
                      + Z_idx_260) + Z_idx_299) + Z_idx_305) + Z_idx_311) +
                  Z_idx_317) + Z_idx_323) + Z_idx_171) - 137.29309999999998) /
    14.0;
  Z_idx_471 = 0.46277 * Z_idx_6;
  Z_idx_467 = 0.33886 * Z_idx_7;
  Z_idx_476 = Z_idx_6 * Z_idx_471 + Z_idx_7 * Z_idx_467;
  Z_idx_505 = 0.23479 * Z_idx_476;
  Z_idx_159_tmp = Z_idx_6 * VAR[8];
  b_Z_idx_159_tmp = Z_idx_13_tmp * VAR[10];
  Z_idx_181_tmp = b_Z_idx_159_tmp - Z_idx_159_tmp;
  *wz_imu = VAR[9] - Z_idx_5 * VAR[10];
  Z_idx_3 = 0.23479 * *wz_imu;
  Z_idx_464 = 0.46277 * Z_idx_181_tmp;
  vrel1_Tx = Z_idx_181_tmp * Z_idx_3 - *wz_imu * Z_idx_464;
  Z_idx_1 = Z_idx_7 * VAR[8];
  Z_idx_158_tmp = Z_idx_10 * VAR[10];
  Z_idx_120_tmp = Z_idx_1 + Z_idx_158_tmp;
  Z_idx_46 = 0.33886 * Z_idx_120_tmp;
  Z_idx_356 = *wz_imu * Z_idx_46 - Z_idx_120_tmp * Z_idx_3;
  Z_idx_17 = Z_idx_0 * VAR[8] * VAR[10];
  Z_idx_361 = 0.23479 * Z_idx_17;
  Z_idx_19 = VAR[10] * Z_idx_18 - Z_idx_159_tmp * VAR[9];
  Z_idx_469 = 0.33886 * Z_idx_19;
  Z_idx_21 = VAR[10] * Z_idx_20 - Z_idx_1 * VAR[9];
  Z_idx_472 = 0.46277 * Z_idx_21;
  Z_idx_58 = Z_idx_120_tmp * Z_idx_464 - Z_idx_181_tmp * Z_idx_46;
  Z_idx_3 = 0.11520666666666668 * Z_idx_2;
  Z_idx_1 = 0.11520666666666668 * Z_idx_4;
  Z_idx_46 = -0.066126666666666667 * Z_idx_2;
  Z_idx_47 = -0.066126666666666667 * Z_idx_4;
  Z_idx_157 = -0.24746 * Z_idx_2;
  Z_idx_152 = -0.24746 * Z_idx_4;
  Z_idx_43 = ((((((-0.5 * (Fn1 * (((((0.36 * Z_idx_8 * vrel1_Ny + 0.189 *
    Z_idx_12 * vrel1_Nx) + 0.29654 * Z_idx_2 * vrel1_Ny) - 0.189 * Z_idx_11 *
    vrel1_Ny) - 0.29654 * Z_idx_4 * vrel1_Nx) - 0.36 * Z_idx_9 * vrel1_Nx)) -
                   0.5 * (Z_idx_263 * (((((0.36 * Z_idx_8 * Z_idx_272 + 0.189 *
    Z_idx_12 * vrel2_Nx) + Z_idx_3 * Z_idx_272) - 0.189 * Z_idx_11 * Z_idx_272)
    - Z_idx_1 * vrel2_Nx) - 0.36 * Z_idx_9 * vrel2_Nx))) - 0.5 * (Z_idx_266 *
    (((((0.36 * Z_idx_8 * vrel3_Ny + 0.189 * Z_idx_12 * vrel3_Nx) + Z_idx_46 *
        vrel3_Ny) - 0.189 * Z_idx_11 * vrel3_Ny) - Z_idx_47 * vrel3_Nx) - 0.36 *
     Z_idx_9 * vrel3_Nx))) - 0.5 * (Fn4 * (((((0.36 * Z_idx_8 * vrel4_Ny + 0.189
    * Z_idx_12 * vrel4_Nx) + Z_idx_157 * vrel4_Ny) - 0.189 * Z_idx_11 * vrel4_Ny)
    - Z_idx_152 * vrel4_Nx) - 0.36 * Z_idx_9 * vrel4_Nx))) - 0.5 * (Fn5 *
    (((((0.36 * Z_idx_9 * vrel5_Nx + 0.189 * Z_idx_12 * vrel5_Nx) + 0.29654 *
        Z_idx_2 * vrel5_Ny) - 0.189 * Z_idx_11 * vrel5_Ny) - 0.29654 * Z_idx_4 *
      vrel5_Nx) - 0.36 * Z_idx_8 * vrel5_Ny))) - 0.5 * (Z_idx_275 * (((((0.36 *
    Z_idx_9 * vrel6_Nx + 0.189 * Z_idx_12 * vrel6_Nx) + Z_idx_3 * vrel5_Tx) -
    0.189 * Z_idx_11 * vrel5_Tx) - Z_idx_1 * vrel6_Nx) - 0.36 * Z_idx_8 *
    vrel5_Tx))) - 0.5 * (Z_idx_278 * (((((0.36 * Z_idx_9 * vrel7_Nx + 0.189 *
    Z_idx_12 * vrel7_Nx) + Z_idx_46 * Z_idx_45) - 0.189 * Z_idx_11 * Z_idx_45) -
    Z_idx_47 * vrel7_Nx) - 0.36 * Z_idx_8 * Z_idx_45))) - 0.5 * (Fn8 * (((((0.36
    * Z_idx_9 * Z_idx_166 + 0.189 * Z_idx_12 * Z_idx_166) + Z_idx_157 * Z_idx_42)
    - 0.189 * Z_idx_11 * Z_idx_42) - Z_idx_152 * Z_idx_166) - 0.36 * Z_idx_8 *
    Z_idx_42));
  Z_idx_44 = Fn1 * (((2.0 * (Z_idx_102 - 0.0268) + 0.36 * Z_idx_10) - 0.189 *
                     Z_idx_13_tmp) - 0.29654 * Z_idx_5);
  Z_idx_3 = 0.11520666666666668 * Z_idx_5;
  Z_idx_332 = Z_idx_263 * (((2.0 * (Z_idx_113 - 0.0268) + 0.36 * Z_idx_10) -
    0.189 * Z_idx_13_tmp) - Z_idx_3);
  Z_idx_1 = -0.066126666666666667 * Z_idx_5;
  Z_idx_69 = Z_idx_266 * (((2.0 * (Z_idx_120 - 0.0268) + 0.36 * Z_idx_10) -
    0.189 * Z_idx_13_tmp) - Z_idx_1);
  Z_idx_47 = -0.24746 * Z_idx_5;
  Z_idx_157 = Fn4 * (((2.0 * (Z_idx_127 - 0.0268) + 0.36 * Z_idx_10) - 0.189 *
                      Z_idx_13_tmp) - Z_idx_47);
  Z_idx_46 = Fn5 * (((2.0 * (Z_idx_133 - 0.0268) - 0.189 * Z_idx_13_tmp) -
                     0.29654 * Z_idx_5) - 0.36 * Z_idx_10);
  Z_idx_464 = Z_idx_275 * (((2.0 * (Z_idx_137 - 0.0268) - 0.189 * Z_idx_13_tmp)
    - Z_idx_3) - 0.36 * Z_idx_10);
  Z_idx_1 = Z_idx_278 * (((2.0 * (Z_idx_141 - 0.0268) - 0.189 * Z_idx_13_tmp) -
    Z_idx_1) - 0.36 * Z_idx_10);
  Z_idx_3 = Fn8 * (((2.0 * (Z_idx_145 - 0.0268) - 0.189 * Z_idx_13_tmp) -
                    Z_idx_47) - 0.36 * Z_idx_10);
  Z_idx_152 = ((((((0.5 * (Z_idx_44 * vrel1_Ny) + 0.5 * (Z_idx_332 * Z_idx_272))
                   + 0.5 * (Z_idx_69 * vrel3_Ny)) + 0.5 * (Z_idx_157 * vrel4_Ny))
                 + 0.5 * (Z_idx_46 * vrel5_Ny)) + 0.5 * (Z_idx_464 * vrel5_Tx))
               + 0.5 * (Z_idx_1 * Z_idx_45)) + 0.5 * (Z_idx_3 * Z_idx_42);
  vrel5_Tx = ((((((-0.5 * (Z_idx_44 * vrel1_Nx) - 0.5 * (Z_idx_332 * vrel2_Nx))
                  - 0.5 * (Z_idx_69 * vrel3_Nx)) - 0.5 * (Z_idx_157 * vrel4_Nx))
                - 0.5 * (Z_idx_46 * vrel5_Nx)) - 0.5 * (Z_idx_464 * vrel6_Nx)) -
              0.5 * (Z_idx_1 * vrel7_Nx)) - 0.5 * (Z_idx_3 * Z_idx_166);
  Z_idx_44 = (((((Z_idx_10 * vrel1_Tx + Z_idx_13_tmp * Z_idx_356) + Z_idx_5 *
                 Z_idx_361) + Z_idx_10 * Z_idx_469) + Z_idx_13_tmp * Z_idx_472)
              - Z_idx_5 * Z_idx_58) - ((((((((((((((((Z_idx_162_tmp * Z_idx_43 +
    Z_idx_165_tmp * Z_idx_152) + b_Z_idx_157_tmp * vrel5_Tx) + b_Z_idx_162_tmp *
    Z_idx_43) + b_Z_idx_165_tmp * Z_idx_152) + c_Z_idx_157_tmp * vrel5_Tx) +
    c_Z_idx_162_tmp * Z_idx_43) - Z_idx_263 * ((Z_idx_163_tmp - Z_idx_184_tmp) -
    b_Z_idx_184_tmp)) - Z_idx_266 * ((Z_idx_163_tmp - Z_idx_194_tmp) -
    b_Z_idx_194_tmp)) - Z_idx_275 * ((Z_idx_217_tmp - Z_idx_184_tmp) -
    Z_idx_232_tmp)) - Z_idx_278 * ((Z_idx_217_tmp - Z_idx_194_tmp) -
    Z_idx_332_tmp)) - Fn1_tmp * ((Z_idx_163_tmp - b_Z_idx_163_tmp) -
    c_Z_idx_163_tmp)) - Fn4_tmp * ((Z_idx_163_tmp - Z_idx_204_tmp) -
    b_Z_idx_204_tmp)) - Z_idx_202_tmp * ((Z_idx_217_tmp - b_Z_idx_163_tmp) -
    b_Z_idx_217_tmp)) - Z_idx_199_tmp * ((Z_idx_217_tmp - Z_idx_204_tmp) -
    Z_idx_183_tmp)) - c_Z_idx_165_tmp * Z_idx_152) - Z_idx_157_tmp * vrel5_Tx);
  Z_idx_42 = 0.23479 * Z_idx_5;
  Z_idx_45 = Z_idx_42 * Z_idx_476;
  Z_idx_59 = Z_idx_0 * (0.36 * Z_idx_7 + 0.189 * Z_idx_6);
  Z_idx_3 = Z_idx_0 * (0.36 * Z_idx_7 - 0.189 * Z_idx_6);
  Z_idx_1 = 0.36 * Z_idx_13_tmp + 0.189 * Z_idx_10;
  Z_idx_157 = 0.36 * Z_idx_13_tmp - 0.189 * Z_idx_10;
  Z_idx_46 = (Z_idx_58 - Z_idx_361) - ((((((((((((((((((0.5 * Z_idx_263 *
    Z_idx_1 + 0.5 * Z_idx_266 * Z_idx_1) + 0.5 * Fn1_tmp * Z_idx_1) + 0.5 *
    Fn4_tmp * Z_idx_1) + Z_idx_2 * Z_idx_152) + Z_idx_4 * vrel5_Tx) + 0.5 *
    Z_idx_59 * Z_idx_287) + 0.5 * Z_idx_59 * Z_idx_260) + 0.5 * Z_idx_59 *
    Z_idx_299) + 0.5 * Z_idx_59 * Z_idx_305) - 0.5 * Z_idx_275 * Z_idx_157) -
    0.5 * Z_idx_278 * Z_idx_157) - 0.5 * Z_idx_202_tmp * Z_idx_157) - 0.5 *
    Z_idx_199_tmp * Z_idx_157) - Z_idx_5 * Z_idx_43) - 0.5 * Z_idx_3 * Z_idx_311)
    - 0.5 * Z_idx_3 * Z_idx_317) - 0.5 * Z_idx_3 * Z_idx_323) - 0.5 * Z_idx_3 *
    Z_idx_171);
  Z_idx_69 = Z_idx_10 * Z_idx_467 - Z_idx_13_tmp * Z_idx_471;
  Z_idx_3 = 0.0945 * Z_idx_5 * Z_idx_7;
  Z_idx_1 = 0.18 * Z_idx_5 * Z_idx_6;
  Z_idx_47 = Z_idx_3 + Z_idx_1;
  Z_idx_152 = (((Z_idx_7 * vrel1_Tx + Z_idx_7 * Z_idx_469) - Z_idx_6 * Z_idx_356)
               - Z_idx_6 * Z_idx_472) - (((((((((((((((((((((((Z_idx_3 -
    Z_idx_103) - Z_idx_1) * Z_idx_287 + ((Z_idx_3 - Z_idx_114) - Z_idx_1) *
    Z_idx_260) + ((Z_idx_3 - Z_idx_121) - Z_idx_1) * Z_idx_299) + ((Z_idx_3 -
    Z_idx_128) - Z_idx_1) * Z_idx_305) + (Z_idx_47 - Z_idx_103) * Z_idx_311) +
    (Z_idx_47 - Z_idx_114) * Z_idx_317) + (Z_idx_47 - Z_idx_121) * Z_idx_323) +
    (Z_idx_47 - Z_idx_128) * Z_idx_171) + Z_idx_164_tmp * Z_idx_152) +
    Z_idx_169_tmp * vrel5_Tx) + Z_idx_7 * Z_idx_10 * Z_idx_43) - Fn1_tmp *
    ((Z_idx_160_tmp + b_Z_idx_160_tmp) + c_Z_idx_160_tmp)) - Z_idx_202_tmp *
    ((Z_idx_215_tmp + b_Z_idx_160_tmp) + c_Z_idx_160_tmp)) - Z_idx_263 *
    ((Z_idx_160_tmp - Z_idx_183_tmp_tmp) - b_Z_idx_183_tmp_tmp)) - Z_idx_266 *
    ((Z_idx_160_tmp - Z_idx_193_tmp_tmp) - b_Z_idx_193_tmp_tmp)) - Z_idx_275 *
    ((Z_idx_215_tmp - Z_idx_183_tmp_tmp) - b_Z_idx_183_tmp_tmp)) - Z_idx_278 *
    ((Z_idx_215_tmp - Z_idx_193_tmp_tmp) - b_Z_idx_193_tmp_tmp)) - Fn4_tmp *
    ((Z_idx_160_tmp - Z_idx_203_tmp_tmp) - b_Z_idx_203_tmp_tmp)) - Z_idx_199_tmp
    * ((Z_idx_215_tmp - Z_idx_203_tmp_tmp) - b_Z_idx_203_tmp_tmp)) -
    b_Z_idx_164_tmp * Z_idx_152) - b_Z_idx_169_tmp * vrel5_Tx) - Z_idx_6 *
    Z_idx_13_tmp * Z_idx_43);
  Z_idx_3 = 0.33886 * Z_idx_10;
  Z_idx_1 = 0.46277 * Z_idx_13_tmp;
  Z_idx_332 = (Z_idx_5 * Z_idx_42 + Z_idx_10 * Z_idx_3) + Z_idx_13_tmp * Z_idx_1;
  Z_idx_1 = Z_idx_7 * Z_idx_3 - Z_idx_6 * Z_idx_1;
  Z_idx_3 = 0.23479 * Z_idx_1;
  Z_idx_157 = (Z_idx_332 * Z_idx_505 - Z_idx_42 * Z_idx_45) - Z_idx_69 * Z_idx_3;
  Z_idx_464 = ((Z_idx_505 * Z_idx_44 + Z_idx_45 * Z_idx_46) - 0.23479 * Z_idx_69
               * Z_idx_152) / Z_idx_157;
  vrel5_Tx = (((0.23479 * Z_idx_332 - Z_idx_42 * Z_idx_42) * Z_idx_152 - Z_idx_3
               * Z_idx_44) - Z_idx_42 * Z_idx_1 * Z_idx_46) / Z_idx_157;
  Z_idx_59 = ((Z_idx_42 * Z_idx_69 * Z_idx_152 - Z_idx_45 * Z_idx_44) -
              (Z_idx_476 * Z_idx_332 - Z_idx_1 * Z_idx_69) * Z_idx_46) /
    Z_idx_157;

  /* =========================================================================== */
  /* =========================================================================== */
  VARp[0] = VAR[8];
  VARp[1] = VAR[9];
  VARp[2] = VAR[10];
  VARp[3] = q_lmDt;
  VARp[4] = q_rmDt;
  VARp[5] = VAR[11];
  VARp[6] = VAR[12];
  VARp[7] = VAR[13];
  VARp[8] = -vrel5_Tx;
  VARp[9] = Z_idx_59;
  VARp[10] = -Z_idx_464;
  VARp[11] = Z_idx_502;
  VARp[12] = Z_idx_503;
  VARp[13] = Z_idx_504;

  /* =========================================================================== */
  /* =========================================================================== */
  *wx_imu = -Z_idx_7 * VAR[8] - Z_idx_158_tmp;
  *wy_imu = Z_idx_159_tmp - b_Z_idx_159_tmp;
  Z_idx_152 = 0.24588 * Z_idx_6;
  Z_idx_46 = 0.08935 * Z_idx_6 + 0.10685 * Z_idx_7;
  Z_idx_47 = 0.10685 * Z_idx_10 - 0.08935 * Z_idx_13_tmp;
  Z_idx_332 = -0.24588 * Z_idx_10 - 0.08935 * Z_idx_5;
  Z_idx_157 = 0.24588 * Z_idx_7;
  Z_idx_1 = Z_idx_46 * VAR[8] + Z_idx_47 * VAR[10];
  Z_idx_69 = (0.08935 * VAR[9] + Z_idx_332 * VAR[10]) - Z_idx_157 * VAR[8];
  Z_idx_3 = 0.24588 * Z_idx_13_tmp + 0.10685 * Z_idx_5;
  *ax_imu = ((((((0.10685 * Z_idx_59 + Z_idx_152 * -vrel5_Tx) - 9.80665 *
                 Z_idx_10) - (((0.24588 * Z_idx_21 + 0.10685 * Z_idx_17) +
    Z_idx_1 * Z_idx_181_tmp) - Z_idx_69 * *wz_imu)) - Z_idx_8 * Z_idx_502) -
              Z_idx_9 * Z_idx_503) - Z_idx_10 * Z_idx_504) - Z_idx_3 *
    -Z_idx_464;
  Z_idx_3 = (Z_idx_3 * VAR[10] - 0.10685 * VAR[9]) - Z_idx_152 * VAR[8];
  *ay_imu = ((((((Z_idx_157 * -vrel5_Tx - 9.80665 * Z_idx_13_tmp) - (((-0.24588 *
    Z_idx_19 - 0.08935 * Z_idx_17) + Z_idx_3 * *wz_imu) - Z_idx_1 *
    Z_idx_120_tmp)) - 0.08935 * Z_idx_59) - Z_idx_11 * Z_idx_502) - Z_idx_12 *
              Z_idx_503) - Z_idx_13_tmp * Z_idx_504) - Z_idx_332 * -Z_idx_464;
  *az_imu = ((((((((0.10685 * Z_idx_19 - 0.08935 * Z_idx_21) + Z_idx_69 *
                   Z_idx_120_tmp) - Z_idx_3 * Z_idx_181_tmp) + Z_idx_2 *
                 Z_idx_502) + Z_idx_4 * Z_idx_503) + Z_idx_46 * -vrel5_Tx) +
              Z_idx_47 * -Z_idx_464) - 9.80665 * Z_idx_5) - Z_idx_5 * Z_idx_504;

  /* ============================== */
  /*  End of function agbot */
  /* ============================== */
}

/* End of code generation (agbot_simulinkblock_20191209.cpp) */
