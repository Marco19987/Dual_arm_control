//
// Academic License - for use in teaching, academic research, and meeting
// course requirements at degree granting institutions only.  Not for
// government, commercial, or other organizational use.
// File: _coder_jacobian_WRh_to_oTg1_oTg2_api.h
//
// MATLAB Coder version            : 23.2
// C/C++ source code generated on  : 11-Apr-2025 10:25:57
//

#ifndef _CODER_JACOBIAN_WRH_TO_OTG1_OTG2_API_H
#define _CODER_JACOBIAN_WRH_TO_OTG1_OTG2_API_H

// Include Files
#include "emlrt.h"
#include "mex.h"
#include "tmwtypes.h"
#include <algorithm>
#include <cstring>

// Variable Declarations
extern emlrtCTX emlrtRootTLSGlobal;
extern emlrtContext emlrtContextGlobal;

// Function Declarations
void jacobian_WRh_to_oTg1_oTg2(real_T in1[3], real_T in2[4], real_T in3[3],
                               real_T in4[3], real_T in5[3], real_T in6[4],
                               real_T in7[3], real_T in8[4], real_T in9[3],
                               real_T in10[4], real_T in11[3], real_T in12[3],
                               real_T in13[3], real_T in14[3], real_T in15[3],
                               real_T in16[4], real_T in17[3], real_T in18[4],
                               real_T in19[3], real_T in20[4], real_T in21[6],
                               real_T in22[6], real_T in23[6], real_T in24[6],
                               real_T b_jacobian_WRh_to_oTg1_oTg2[84]);

void jacobian_WRh_to_oTg1_oTg2_api(const mxArray *const prhs[24],
                                   const mxArray **plhs);

void jacobian_WRh_to_oTg1_oTg2_atexit();

void jacobian_WRh_to_oTg1_oTg2_initialize();

void jacobian_WRh_to_oTg1_oTg2_terminate();

void jacobian_WRh_to_oTg1_oTg2_xil_shutdown();

void jacobian_WRh_to_oTg1_oTg2_xil_terminate();

void jacobian_h_to_b2Tb1(real_T in1[3], real_T in2[4], real_T in3[3],
                         real_T in4[3], real_T in5[3], real_T in6[4],
                         real_T in7[3], real_T in8[4], real_T in9[3],
                         real_T in10[4], real_T in11[3], real_T in12[3],
                         real_T in13[3], real_T in14[3], real_T in15[3],
                         real_T in16[4], real_T in17[3], real_T in18[4],
                         real_T in19[3], real_T in20[4], real_T in21[6],
                         real_T in22[6], real_T in23[6], real_T in24[6],
                         real_T b_jacobian_h_to_b2Tb1[84]);

void jacobian_h_to_b2Tb1_api(const mxArray *const prhs[24],
                             const mxArray **plhs);

void jacobian_h_to_oTg1_oTg2(real_T in1[3], real_T in2[4], real_T in3[3],
                             real_T in4[3], real_T in5[3], real_T in6[4],
                             real_T in7[3], real_T in8[4], real_T in9[3],
                             real_T in10[4], real_T in11[3], real_T in12[3],
                             real_T in13[3], real_T in14[3], real_T in15[3],
                             real_T in16[4], real_T in17[3], real_T in18[4],
                             real_T in19[3], real_T in20[4], real_T in21[6],
                             real_T in22[6], real_T in23[6], real_T in24[6],
                             real_T b_jacobian_h_to_oTg1_oTg2[168]);

void jacobian_h_to_oTg1_oTg2_api(const mxArray *const prhs[24],
                                 const mxArray **plhs);

void jacobian_h_to_x_state_not_ext(
    real_T in1[3], real_T in2[4], real_T in3[3], real_T in4[3], real_T in5[3],
    real_T in6[4], real_T in7[3], real_T in8[4], real_T in9[3], real_T in10[4],
    real_T in11[3], real_T in12[3], real_T in13[3], real_T in14[3],
    real_T in15[3], real_T in16[4], real_T in17[3], real_T in18[4],
    real_T in19[3], real_T in20[4], real_T in21[6], real_T in22[6],
    real_T in23[6], real_T in24[6],
    real_T b_jacobian_h_to_x_state_not_ext[324]);

void jacobian_h_to_x_state_not_ext_api(const mxArray *const prhs[24],
                                       const mxArray **plhs);

#endif
//
// File trailer for _coder_jacobian_WRh_to_oTg1_oTg2_api.h
//
// [EOF]
//
