//
// Academic License - for use in teaching, academic research, and meeting
// course requirements at degree granting institutions only.  Not for
// government, commercial, or other organizational use.
// File: _coder_jacobian_WRh_to_oTg1_oTg2_mex.h
//
// MATLAB Coder version            : 23.2
// C/C++ source code generated on  : 11-Apr-2025 10:25:57
//

#ifndef _CODER_JACOBIAN_WRH_TO_OTG1_OTG2_MEX_H
#define _CODER_JACOBIAN_WRH_TO_OTG1_OTG2_MEX_H

// Include Files
#include "emlrt.h"
#include "mex.h"
#include "tmwtypes.h"

// Function Declarations
MEXFUNCTION_LINKAGE void mexFunction(int32_T nlhs, mxArray *plhs[],
                                     int32_T nrhs, const mxArray *prhs[]);

emlrtCTX mexFunctionCreateRootTLS();

void unsafe_jacobian_WRh_to_oTg1_oTg2_mexFunction(int32_T nlhs,
                                                  mxArray *plhs[1],
                                                  int32_T nrhs,
                                                  const mxArray *prhs[24]);

void unsafe_jacobian_h_to_b2Tb1_mexFunction(int32_T nlhs, mxArray *plhs[1],
                                            int32_T nrhs,
                                            const mxArray *prhs[24]);

void unsafe_jacobian_h_to_oTg1_oTg2_mexFunction(int32_T nlhs, mxArray *plhs[1],
                                                int32_T nrhs,
                                                const mxArray *prhs[24]);

void unsafe_jacobian_h_to_x_state_not_ext_mexFunction(int32_T nlhs,
                                                      mxArray *plhs[1],
                                                      int32_T nrhs,
                                                      const mxArray *prhs[24]);

#endif
//
// File trailer for _coder_jacobian_WRh_to_oTg1_oTg2_mex.h
//
// [EOF]
//
