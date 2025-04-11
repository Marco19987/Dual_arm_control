//
// Academic License - for use in teaching, academic research, and meeting
// course requirements at degree granting institutions only.  Not for
// government, commercial, or other organizational use.
// File: _coder_jacobian_WRh_to_oTg1_oTg2_mex.cpp
//
// MATLAB Coder version            : 23.2
// C/C++ source code generated on  : 11-Apr-2025 10:25:57
//

// Include Files
#include "_coder_jacobian_WRh_to_oTg1_oTg2_mex.h"
#include "_coder_jacobian_WRh_to_oTg1_oTg2_api.h"

// Function Definitions
//
// Arguments    : int32_T nlhs
//                mxArray *plhs[]
//                int32_T nrhs
//                const mxArray *prhs[]
// Return Type  : void
//
void mexFunction(int32_T nlhs, mxArray *plhs[], int32_T nrhs,
                 const mxArray *prhs[])
{
  static const char_T *emlrtEntryPoints[4]{
      "jacobian_h_to_b2Tb1", "jacobian_h_to_oTg1_oTg2",
      "jacobian_h_to_x_state_not_ext", "jacobian_WRh_to_oTg1_oTg2"};
  emlrtStack st{
      nullptr, // site
      nullptr, // tls
      nullptr  // prev
  };
  mexAtExit(&jacobian_WRh_to_oTg1_oTg2_atexit);
  // Module initialization.
  jacobian_WRh_to_oTg1_oTg2_initialize();
  st.tls = emlrtRootTLSGlobal;
  // Dispatch the entry-point.
  switch (emlrtGetEntryPointIndexR2016a(
      &st, nrhs, &prhs[0], (const char_T **)&emlrtEntryPoints[0], 4)) {
  case 0:
    unsafe_jacobian_h_to_b2Tb1_mexFunction(nlhs, plhs, nrhs - 1, &prhs[1]);
    break;
  case 1:
    unsafe_jacobian_h_to_oTg1_oTg2_mexFunction(nlhs, plhs, nrhs - 1, &prhs[1]);
    break;
  case 2:
    unsafe_jacobian_h_to_x_state_not_ext_mexFunction(nlhs, plhs, nrhs - 1,
                                                     &prhs[1]);
    break;
  case 3:
    unsafe_jacobian_WRh_to_oTg1_oTg2_mexFunction(nlhs, plhs, nrhs - 1,
                                                 &prhs[1]);
    break;
  }
  // Module termination.
  jacobian_WRh_to_oTg1_oTg2_terminate();
}

//
// Arguments    : void
// Return Type  : emlrtCTX
//
emlrtCTX mexFunctionCreateRootTLS()
{
  emlrtCreateRootTLSR2022a(&emlrtRootTLSGlobal, &emlrtContextGlobal, nullptr, 1,
                           nullptr, "UTF-8", true);
  return emlrtRootTLSGlobal;
}

//
// Arguments    : int32_T nlhs
//                mxArray *plhs[1]
//                int32_T nrhs
//                const mxArray *prhs[24]
// Return Type  : void
//
void unsafe_jacobian_WRh_to_oTg1_oTg2_mexFunction(int32_T nlhs,
                                                  mxArray *plhs[1],
                                                  int32_T nrhs,
                                                  const mxArray *prhs[24])
{
  emlrtStack st{
      nullptr, // site
      nullptr, // tls
      nullptr  // prev
  };
  const mxArray *b_prhs[24];
  const mxArray *outputs;
  st.tls = emlrtRootTLSGlobal;
  // Check for proper number of arguments.
  if (nrhs != 24) {
    emlrtErrMsgIdAndTxt(&st, "EMLRT:runTime:WrongNumberOfInputs", 5, 12, 24, 4,
                        25, "jacobian_WRh_to_oTg1_oTg2");
  }
  if (nlhs > 1) {
    emlrtErrMsgIdAndTxt(&st, "EMLRT:runTime:TooManyOutputArguments", 3, 4, 25,
                        "jacobian_WRh_to_oTg1_oTg2");
  }
  // Call the function.
  for (int32_T i{0}; i < 24; i++) {
    b_prhs[i] = prhs[i];
  }
  jacobian_WRh_to_oTg1_oTg2_api(b_prhs, &outputs);
  // Copy over outputs to the caller.
  emlrtReturnArrays(1, &plhs[0], &outputs);
}

//
// Arguments    : int32_T nlhs
//                mxArray *plhs[1]
//                int32_T nrhs
//                const mxArray *prhs[24]
// Return Type  : void
//
void unsafe_jacobian_h_to_b2Tb1_mexFunction(int32_T nlhs, mxArray *plhs[1],
                                            int32_T nrhs,
                                            const mxArray *prhs[24])
{
  emlrtStack st{
      nullptr, // site
      nullptr, // tls
      nullptr  // prev
  };
  const mxArray *b_prhs[24];
  const mxArray *outputs;
  st.tls = emlrtRootTLSGlobal;
  // Check for proper number of arguments.
  if (nrhs != 24) {
    emlrtErrMsgIdAndTxt(&st, "EMLRT:runTime:WrongNumberOfInputs", 5, 12, 24, 4,
                        19, "jacobian_h_to_b2Tb1");
  }
  if (nlhs > 1) {
    emlrtErrMsgIdAndTxt(&st, "EMLRT:runTime:TooManyOutputArguments", 3, 4, 19,
                        "jacobian_h_to_b2Tb1");
  }
  // Call the function.
  for (int32_T i{0}; i < 24; i++) {
    b_prhs[i] = prhs[i];
  }
  jacobian_h_to_b2Tb1_api(b_prhs, &outputs);
  // Copy over outputs to the caller.
  emlrtReturnArrays(1, &plhs[0], &outputs);
}

//
// Arguments    : int32_T nlhs
//                mxArray *plhs[1]
//                int32_T nrhs
//                const mxArray *prhs[24]
// Return Type  : void
//
void unsafe_jacobian_h_to_oTg1_oTg2_mexFunction(int32_T nlhs, mxArray *plhs[1],
                                                int32_T nrhs,
                                                const mxArray *prhs[24])
{
  emlrtStack st{
      nullptr, // site
      nullptr, // tls
      nullptr  // prev
  };
  const mxArray *b_prhs[24];
  const mxArray *outputs;
  st.tls = emlrtRootTLSGlobal;
  // Check for proper number of arguments.
  if (nrhs != 24) {
    emlrtErrMsgIdAndTxt(&st, "EMLRT:runTime:WrongNumberOfInputs", 5, 12, 24, 4,
                        23, "jacobian_h_to_oTg1_oTg2");
  }
  if (nlhs > 1) {
    emlrtErrMsgIdAndTxt(&st, "EMLRT:runTime:TooManyOutputArguments", 3, 4, 23,
                        "jacobian_h_to_oTg1_oTg2");
  }
  // Call the function.
  for (int32_T i{0}; i < 24; i++) {
    b_prhs[i] = prhs[i];
  }
  jacobian_h_to_oTg1_oTg2_api(b_prhs, &outputs);
  // Copy over outputs to the caller.
  emlrtReturnArrays(1, &plhs[0], &outputs);
}

//
// Arguments    : int32_T nlhs
//                mxArray *plhs[1]
//                int32_T nrhs
//                const mxArray *prhs[24]
// Return Type  : void
//
void unsafe_jacobian_h_to_x_state_not_ext_mexFunction(int32_T nlhs,
                                                      mxArray *plhs[1],
                                                      int32_T nrhs,
                                                      const mxArray *prhs[24])
{
  emlrtStack st{
      nullptr, // site
      nullptr, // tls
      nullptr  // prev
  };
  const mxArray *b_prhs[24];
  const mxArray *outputs;
  st.tls = emlrtRootTLSGlobal;
  // Check for proper number of arguments.
  if (nrhs != 24) {
    emlrtErrMsgIdAndTxt(&st, "EMLRT:runTime:WrongNumberOfInputs", 5, 12, 24, 4,
                        29, "jacobian_h_to_x_state_not_ext");
  }
  if (nlhs > 1) {
    emlrtErrMsgIdAndTxt(&st, "EMLRT:runTime:TooManyOutputArguments", 3, 4, 29,
                        "jacobian_h_to_x_state_not_ext");
  }
  // Call the function.
  for (int32_T i{0}; i < 24; i++) {
    b_prhs[i] = prhs[i];
  }
  jacobian_h_to_x_state_not_ext_api(b_prhs, &outputs);
  // Copy over outputs to the caller.
  emlrtReturnArrays(1, &plhs[0], &outputs);
}

//
// File trailer for _coder_jacobian_WRh_to_oTg1_oTg2_mex.cpp
//
// [EOF]
//
