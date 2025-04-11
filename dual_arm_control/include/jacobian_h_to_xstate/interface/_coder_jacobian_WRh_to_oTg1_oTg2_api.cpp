//
// Academic License - for use in teaching, academic research, and meeting
// course requirements at degree granting institutions only.  Not for
// government, commercial, or other organizational use.
// File: _coder_jacobian_WRh_to_oTg1_oTg2_api.cpp
//
// MATLAB Coder version            : 23.2
// C/C++ source code generated on  : 11-Apr-2025 10:25:57
//

// Include Files
#include "_coder_jacobian_WRh_to_oTg1_oTg2_api.h"
#include "_coder_jacobian_WRh_to_oTg1_oTg2_mex.h"

// Variable Definitions
emlrtCTX emlrtRootTLSGlobal{nullptr};

emlrtContext emlrtContextGlobal{
    true,                                                 // bFirstTime
    false,                                                // bInitialized
    131643U,                                              // fVersionInfo
    nullptr,                                              // fErrorFunction
    "jacobian_WRh_to_oTg1_oTg2",                          // fFunctionName
    nullptr,                                              // fRTCallStack
    false,                                                // bDebugMode
    {2045744189U, 2170104910U, 2743257031U, 4284093946U}, // fSigWrd
    nullptr                                               // fSigMem
};

// Function Declarations
static real_T (*b_emlrt_marshallIn(const emlrtStack &sp,
                                   const mxArray *b_nullptr,
                                   const char_T *identifier))[4];

static real_T (*b_emlrt_marshallIn(const emlrtStack &sp, const mxArray *u,
                                   const emlrtMsgIdentifier *parentId))[4];

static const mxArray *b_emlrt_marshallOut(const real_T u[168]);

static real_T (*c_emlrt_marshallIn(const emlrtStack &sp,
                                   const mxArray *b_nullptr,
                                   const char_T *identifier))[6];

static real_T (*c_emlrt_marshallIn(const emlrtStack &sp, const mxArray *u,
                                   const emlrtMsgIdentifier *parentId))[6];

static const mxArray *c_emlrt_marshallOut(const real_T u[324]);

static real_T (*d_emlrt_marshallIn(const emlrtStack &sp, const mxArray *src,
                                   const emlrtMsgIdentifier *msgId))[3];

static const mxArray *d_emlrt_marshallOut(const real_T u[84]);

static real_T (*e_emlrt_marshallIn(const emlrtStack &sp, const mxArray *src,
                                   const emlrtMsgIdentifier *msgId))[4];

static real_T (*emlrt_marshallIn(const emlrtStack &sp, const mxArray *b_nullptr,
                                 const char_T *identifier))[3];

static real_T (*emlrt_marshallIn(const emlrtStack &sp, const mxArray *u,
                                 const emlrtMsgIdentifier *parentId))[3];

static const mxArray *emlrt_marshallOut(const real_T u[84]);

static real_T (*f_emlrt_marshallIn(const emlrtStack &sp, const mxArray *src,
                                   const emlrtMsgIdentifier *msgId))[6];

// Function Definitions
//
// Arguments    : const emlrtStack &sp
//                const mxArray *u
//                const emlrtMsgIdentifier *parentId
// Return Type  : real_T (*)[4]
//
static real_T (*b_emlrt_marshallIn(const emlrtStack &sp, const mxArray *u,
                                   const emlrtMsgIdentifier *parentId))[4]
{
  real_T(*y)[4];
  y = e_emlrt_marshallIn(sp, emlrtAlias(u), parentId);
  emlrtDestroyArray(&u);
  return y;
}

//
// Arguments    : const emlrtStack &sp
//                const mxArray *b_nullptr
//                const char_T *identifier
// Return Type  : real_T (*)[4]
//
static real_T (*b_emlrt_marshallIn(const emlrtStack &sp,
                                   const mxArray *b_nullptr,
                                   const char_T *identifier))[4]
{
  emlrtMsgIdentifier thisId;
  real_T(*y)[4];
  thisId.fIdentifier = const_cast<const char_T *>(identifier);
  thisId.fParent = nullptr;
  thisId.bParentIsCell = false;
  y = b_emlrt_marshallIn(sp, emlrtAlias(b_nullptr), &thisId);
  emlrtDestroyArray(&b_nullptr);
  return y;
}

//
// Arguments    : const real_T u[168]
// Return Type  : const mxArray *
//
static const mxArray *b_emlrt_marshallOut(const real_T u[168])
{
  static const int32_T iv[2]{0, 0};
  static const int32_T iv1[2]{12, 14};
  const mxArray *m;
  const mxArray *y;
  y = nullptr;
  m = emlrtCreateNumericArray(2, (const void *)&iv[0], mxDOUBLE_CLASS, mxREAL);
  emlrtMxSetData((mxArray *)m, (void *)&u[0]);
  emlrtSetDimensions((mxArray *)m, &iv1[0], 2);
  emlrtAssign(&y, m);
  return y;
}

//
// Arguments    : const emlrtStack &sp
//                const mxArray *u
//                const emlrtMsgIdentifier *parentId
// Return Type  : real_T (*)[6]
//
static real_T (*c_emlrt_marshallIn(const emlrtStack &sp, const mxArray *u,
                                   const emlrtMsgIdentifier *parentId))[6]
{
  real_T(*y)[6];
  y = f_emlrt_marshallIn(sp, emlrtAlias(u), parentId);
  emlrtDestroyArray(&u);
  return y;
}

//
// Arguments    : const emlrtStack &sp
//                const mxArray *b_nullptr
//                const char_T *identifier
// Return Type  : real_T (*)[6]
//
static real_T (*c_emlrt_marshallIn(const emlrtStack &sp,
                                   const mxArray *b_nullptr,
                                   const char_T *identifier))[6]
{
  emlrtMsgIdentifier thisId;
  real_T(*y)[6];
  thisId.fIdentifier = const_cast<const char_T *>(identifier);
  thisId.fParent = nullptr;
  thisId.bParentIsCell = false;
  y = c_emlrt_marshallIn(sp, emlrtAlias(b_nullptr), &thisId);
  emlrtDestroyArray(&b_nullptr);
  return y;
}

//
// Arguments    : const real_T u[324]
// Return Type  : const mxArray *
//
static const mxArray *c_emlrt_marshallOut(const real_T u[324])
{
  static const int32_T iv[2]{0, 0};
  static const int32_T iv1[2]{12, 27};
  const mxArray *m;
  const mxArray *y;
  y = nullptr;
  m = emlrtCreateNumericArray(2, (const void *)&iv[0], mxDOUBLE_CLASS, mxREAL);
  emlrtMxSetData((mxArray *)m, (void *)&u[0]);
  emlrtSetDimensions((mxArray *)m, &iv1[0], 2);
  emlrtAssign(&y, m);
  return y;
}

//
// Arguments    : const emlrtStack &sp
//                const mxArray *src
//                const emlrtMsgIdentifier *msgId
// Return Type  : real_T (*)[3]
//
static real_T (*d_emlrt_marshallIn(const emlrtStack &sp, const mxArray *src,
                                   const emlrtMsgIdentifier *msgId))[3]
{
  static const int32_T dims{3};
  real_T(*ret)[3];
  int32_T i;
  boolean_T b{false};
  emlrtCheckVsBuiltInR2012b((emlrtConstCTX)&sp, msgId, src, "double", false, 1U,
                            (const void *)&dims, &b, &i);
  ret = (real_T(*)[3])emlrtMxGetData(src);
  emlrtDestroyArray(&src);
  return ret;
}

//
// Arguments    : const real_T u[84]
// Return Type  : const mxArray *
//
static const mxArray *d_emlrt_marshallOut(const real_T u[84])
{
  static const int32_T iv[2]{0, 0};
  static const int32_T iv1[2]{6, 14};
  const mxArray *m;
  const mxArray *y;
  y = nullptr;
  m = emlrtCreateNumericArray(2, (const void *)&iv[0], mxDOUBLE_CLASS, mxREAL);
  emlrtMxSetData((mxArray *)m, (void *)&u[0]);
  emlrtSetDimensions((mxArray *)m, &iv1[0], 2);
  emlrtAssign(&y, m);
  return y;
}

//
// Arguments    : const emlrtStack &sp
//                const mxArray *src
//                const emlrtMsgIdentifier *msgId
// Return Type  : real_T (*)[4]
//
static real_T (*e_emlrt_marshallIn(const emlrtStack &sp, const mxArray *src,
                                   const emlrtMsgIdentifier *msgId))[4]
{
  static const int32_T dims{4};
  real_T(*ret)[4];
  int32_T i;
  boolean_T b{false};
  emlrtCheckVsBuiltInR2012b((emlrtConstCTX)&sp, msgId, src, "double", false, 1U,
                            (const void *)&dims, &b, &i);
  ret = (real_T(*)[4])emlrtMxGetData(src);
  emlrtDestroyArray(&src);
  return ret;
}

//
// Arguments    : const emlrtStack &sp
//                const mxArray *u
//                const emlrtMsgIdentifier *parentId
// Return Type  : real_T (*)[3]
//
static real_T (*emlrt_marshallIn(const emlrtStack &sp, const mxArray *u,
                                 const emlrtMsgIdentifier *parentId))[3]
{
  real_T(*y)[3];
  y = d_emlrt_marshallIn(sp, emlrtAlias(u), parentId);
  emlrtDestroyArray(&u);
  return y;
}

//
// Arguments    : const emlrtStack &sp
//                const mxArray *b_nullptr
//                const char_T *identifier
// Return Type  : real_T (*)[3]
//
static real_T (*emlrt_marshallIn(const emlrtStack &sp, const mxArray *b_nullptr,
                                 const char_T *identifier))[3]
{
  emlrtMsgIdentifier thisId;
  real_T(*y)[3];
  thisId.fIdentifier = const_cast<const char_T *>(identifier);
  thisId.fParent = nullptr;
  thisId.bParentIsCell = false;
  y = emlrt_marshallIn(sp, emlrtAlias(b_nullptr), &thisId);
  emlrtDestroyArray(&b_nullptr);
  return y;
}

//
// Arguments    : const real_T u[84]
// Return Type  : const mxArray *
//
static const mxArray *emlrt_marshallOut(const real_T u[84])
{
  static const int32_T iv[2]{0, 0};
  static const int32_T iv1[2]{12, 7};
  const mxArray *m;
  const mxArray *y;
  y = nullptr;
  m = emlrtCreateNumericArray(2, (const void *)&iv[0], mxDOUBLE_CLASS, mxREAL);
  emlrtMxSetData((mxArray *)m, (void *)&u[0]);
  emlrtSetDimensions((mxArray *)m, &iv1[0], 2);
  emlrtAssign(&y, m);
  return y;
}

//
// Arguments    : const emlrtStack &sp
//                const mxArray *src
//                const emlrtMsgIdentifier *msgId
// Return Type  : real_T (*)[6]
//
static real_T (*f_emlrt_marshallIn(const emlrtStack &sp, const mxArray *src,
                                   const emlrtMsgIdentifier *msgId))[6]
{
  static const int32_T dims{6};
  real_T(*ret)[6];
  int32_T i;
  boolean_T b{false};
  emlrtCheckVsBuiltInR2012b((emlrtConstCTX)&sp, msgId, src, "double", false, 1U,
                            (const void *)&dims, &b, &i);
  ret = (real_T(*)[6])emlrtMxGetData(src);
  emlrtDestroyArray(&src);
  return ret;
}

//
// Arguments    : const mxArray * const prhs[24]
//                const mxArray **plhs
// Return Type  : void
//
void jacobian_WRh_to_oTg1_oTg2_api(const mxArray *const prhs[24],
                                   const mxArray **plhs)
{
  emlrtStack st{
      nullptr, // site
      nullptr, // tls
      nullptr  // prev
  };
  real_T(*b_jacobian_WRh_to_oTg1_oTg2)[84];
  real_T(*in21)[6];
  real_T(*in22)[6];
  real_T(*in23)[6];
  real_T(*in24)[6];
  real_T(*in10)[4];
  real_T(*in16)[4];
  real_T(*in18)[4];
  real_T(*in2)[4];
  real_T(*in20)[4];
  real_T(*in6)[4];
  real_T(*in8)[4];
  real_T(*in1)[3];
  real_T(*in11)[3];
  real_T(*in12)[3];
  real_T(*in13)[3];
  real_T(*in14)[3];
  real_T(*in15)[3];
  real_T(*in17)[3];
  real_T(*in19)[3];
  real_T(*in3)[3];
  real_T(*in4)[3];
  real_T(*in5)[3];
  real_T(*in7)[3];
  real_T(*in9)[3];
  st.tls = emlrtRootTLSGlobal;
  b_jacobian_WRh_to_oTg1_oTg2 = (real_T(*)[84])mxMalloc(sizeof(real_T[84]));
  // Marshall function inputs
  in1 = emlrt_marshallIn(st, emlrtAlias(prhs[0]), "in1");
  in2 = b_emlrt_marshallIn(st, emlrtAlias(prhs[1]), "in2");
  in3 = emlrt_marshallIn(st, emlrtAlias(prhs[2]), "in3");
  in4 = emlrt_marshallIn(st, emlrtAlias(prhs[3]), "in4");
  in5 = emlrt_marshallIn(st, emlrtAlias(prhs[4]), "in5");
  in6 = b_emlrt_marshallIn(st, emlrtAlias(prhs[5]), "in6");
  in7 = emlrt_marshallIn(st, emlrtAlias(prhs[6]), "in7");
  in8 = b_emlrt_marshallIn(st, emlrtAlias(prhs[7]), "in8");
  in9 = emlrt_marshallIn(st, emlrtAlias(prhs[8]), "in9");
  in10 = b_emlrt_marshallIn(st, emlrtAlias(prhs[9]), "in10");
  in11 = emlrt_marshallIn(st, emlrtAlias(prhs[10]), "in11");
  in12 = emlrt_marshallIn(st, emlrtAlias(prhs[11]), "in12");
  in13 = emlrt_marshallIn(st, emlrtAlias(prhs[12]), "in13");
  in14 = emlrt_marshallIn(st, emlrtAlias(prhs[13]), "in14");
  in15 = emlrt_marshallIn(st, emlrtAlias(prhs[14]), "in15");
  in16 = b_emlrt_marshallIn(st, emlrtAlias(prhs[15]), "in16");
  in17 = emlrt_marshallIn(st, emlrtAlias(prhs[16]), "in17");
  in18 = b_emlrt_marshallIn(st, emlrtAlias(prhs[17]), "in18");
  in19 = emlrt_marshallIn(st, emlrtAlias(prhs[18]), "in19");
  in20 = b_emlrt_marshallIn(st, emlrtAlias(prhs[19]), "in20");
  in21 = c_emlrt_marshallIn(st, emlrtAlias(prhs[20]), "in21");
  in22 = c_emlrt_marshallIn(st, emlrtAlias(prhs[21]), "in22");
  in23 = c_emlrt_marshallIn(st, emlrtAlias(prhs[22]), "in23");
  in24 = c_emlrt_marshallIn(st, emlrtAlias(prhs[23]), "in24");
  // Invoke the target function
  jacobian_WRh_to_oTg1_oTg2(*in1, *in2, *in3, *in4, *in5, *in6, *in7, *in8,
                            *in9, *in10, *in11, *in12, *in13, *in14, *in15,
                            *in16, *in17, *in18, *in19, *in20, *in21, *in22,
                            *in23, *in24, *b_jacobian_WRh_to_oTg1_oTg2);
  // Marshall function outputs
  *plhs = d_emlrt_marshallOut(*b_jacobian_WRh_to_oTg1_oTg2);
}

//
// Arguments    : void
// Return Type  : void
//
void jacobian_WRh_to_oTg1_oTg2_atexit()
{
  emlrtStack st{
      nullptr, // site
      nullptr, // tls
      nullptr  // prev
  };
  mexFunctionCreateRootTLS();
  st.tls = emlrtRootTLSGlobal;
  emlrtEnterRtStackR2012b(&st);
  emlrtDestroyRootTLS(&emlrtRootTLSGlobal);
  jacobian_WRh_to_oTg1_oTg2_xil_terminate();
  jacobian_WRh_to_oTg1_oTg2_xil_shutdown();
  emlrtExitTimeCleanup(&emlrtContextGlobal);
}

//
// Arguments    : void
// Return Type  : void
//
void jacobian_WRh_to_oTg1_oTg2_initialize()
{
  emlrtStack st{
      nullptr, // site
      nullptr, // tls
      nullptr  // prev
  };
  mexFunctionCreateRootTLS();
  st.tls = emlrtRootTLSGlobal;
  emlrtClearAllocCountR2012b(&st, false, 0U, nullptr);
  emlrtEnterRtStackR2012b(&st);
  emlrtFirstTimeR2012b(emlrtRootTLSGlobal);
}

//
// Arguments    : void
// Return Type  : void
//
void jacobian_WRh_to_oTg1_oTg2_terminate()
{
  emlrtDestroyRootTLS(&emlrtRootTLSGlobal);
}

//
// Arguments    : const mxArray * const prhs[24]
//                const mxArray **plhs
// Return Type  : void
//
void jacobian_h_to_b2Tb1_api(const mxArray *const prhs[24],
                             const mxArray **plhs)
{
  emlrtStack st{
      nullptr, // site
      nullptr, // tls
      nullptr  // prev
  };
  real_T(*b_jacobian_h_to_b2Tb1)[84];
  real_T(*in21)[6];
  real_T(*in22)[6];
  real_T(*in23)[6];
  real_T(*in24)[6];
  real_T(*in10)[4];
  real_T(*in16)[4];
  real_T(*in18)[4];
  real_T(*in2)[4];
  real_T(*in20)[4];
  real_T(*in6)[4];
  real_T(*in8)[4];
  real_T(*in1)[3];
  real_T(*in11)[3];
  real_T(*in12)[3];
  real_T(*in13)[3];
  real_T(*in14)[3];
  real_T(*in15)[3];
  real_T(*in17)[3];
  real_T(*in19)[3];
  real_T(*in3)[3];
  real_T(*in4)[3];
  real_T(*in5)[3];
  real_T(*in7)[3];
  real_T(*in9)[3];
  st.tls = emlrtRootTLSGlobal;
  b_jacobian_h_to_b2Tb1 = (real_T(*)[84])mxMalloc(sizeof(real_T[84]));
  // Marshall function inputs
  in1 = emlrt_marshallIn(st, emlrtAlias(prhs[0]), "in1");
  in2 = b_emlrt_marshallIn(st, emlrtAlias(prhs[1]), "in2");
  in3 = emlrt_marshallIn(st, emlrtAlias(prhs[2]), "in3");
  in4 = emlrt_marshallIn(st, emlrtAlias(prhs[3]), "in4");
  in5 = emlrt_marshallIn(st, emlrtAlias(prhs[4]), "in5");
  in6 = b_emlrt_marshallIn(st, emlrtAlias(prhs[5]), "in6");
  in7 = emlrt_marshallIn(st, emlrtAlias(prhs[6]), "in7");
  in8 = b_emlrt_marshallIn(st, emlrtAlias(prhs[7]), "in8");
  in9 = emlrt_marshallIn(st, emlrtAlias(prhs[8]), "in9");
  in10 = b_emlrt_marshallIn(st, emlrtAlias(prhs[9]), "in10");
  in11 = emlrt_marshallIn(st, emlrtAlias(prhs[10]), "in11");
  in12 = emlrt_marshallIn(st, emlrtAlias(prhs[11]), "in12");
  in13 = emlrt_marshallIn(st, emlrtAlias(prhs[12]), "in13");
  in14 = emlrt_marshallIn(st, emlrtAlias(prhs[13]), "in14");
  in15 = emlrt_marshallIn(st, emlrtAlias(prhs[14]), "in15");
  in16 = b_emlrt_marshallIn(st, emlrtAlias(prhs[15]), "in16");
  in17 = emlrt_marshallIn(st, emlrtAlias(prhs[16]), "in17");
  in18 = b_emlrt_marshallIn(st, emlrtAlias(prhs[17]), "in18");
  in19 = emlrt_marshallIn(st, emlrtAlias(prhs[18]), "in19");
  in20 = b_emlrt_marshallIn(st, emlrtAlias(prhs[19]), "in20");
  in21 = c_emlrt_marshallIn(st, emlrtAlias(prhs[20]), "in21");
  in22 = c_emlrt_marshallIn(st, emlrtAlias(prhs[21]), "in22");
  in23 = c_emlrt_marshallIn(st, emlrtAlias(prhs[22]), "in23");
  in24 = c_emlrt_marshallIn(st, emlrtAlias(prhs[23]), "in24");
  // Invoke the target function
  jacobian_h_to_b2Tb1(*in1, *in2, *in3, *in4, *in5, *in6, *in7, *in8, *in9,
                      *in10, *in11, *in12, *in13, *in14, *in15, *in16, *in17,
                      *in18, *in19, *in20, *in21, *in22, *in23, *in24,
                      *b_jacobian_h_to_b2Tb1);
  // Marshall function outputs
  *plhs = emlrt_marshallOut(*b_jacobian_h_to_b2Tb1);
}

//
// Arguments    : const mxArray * const prhs[24]
//                const mxArray **plhs
// Return Type  : void
//
void jacobian_h_to_oTg1_oTg2_api(const mxArray *const prhs[24],
                                 const mxArray **plhs)
{
  emlrtStack st{
      nullptr, // site
      nullptr, // tls
      nullptr  // prev
  };
  real_T(*b_jacobian_h_to_oTg1_oTg2)[168];
  real_T(*in21)[6];
  real_T(*in22)[6];
  real_T(*in23)[6];
  real_T(*in24)[6];
  real_T(*in10)[4];
  real_T(*in16)[4];
  real_T(*in18)[4];
  real_T(*in2)[4];
  real_T(*in20)[4];
  real_T(*in6)[4];
  real_T(*in8)[4];
  real_T(*in1)[3];
  real_T(*in11)[3];
  real_T(*in12)[3];
  real_T(*in13)[3];
  real_T(*in14)[3];
  real_T(*in15)[3];
  real_T(*in17)[3];
  real_T(*in19)[3];
  real_T(*in3)[3];
  real_T(*in4)[3];
  real_T(*in5)[3];
  real_T(*in7)[3];
  real_T(*in9)[3];
  st.tls = emlrtRootTLSGlobal;
  b_jacobian_h_to_oTg1_oTg2 = (real_T(*)[168])mxMalloc(sizeof(real_T[168]));
  // Marshall function inputs
  in1 = emlrt_marshallIn(st, emlrtAlias(prhs[0]), "in1");
  in2 = b_emlrt_marshallIn(st, emlrtAlias(prhs[1]), "in2");
  in3 = emlrt_marshallIn(st, emlrtAlias(prhs[2]), "in3");
  in4 = emlrt_marshallIn(st, emlrtAlias(prhs[3]), "in4");
  in5 = emlrt_marshallIn(st, emlrtAlias(prhs[4]), "in5");
  in6 = b_emlrt_marshallIn(st, emlrtAlias(prhs[5]), "in6");
  in7 = emlrt_marshallIn(st, emlrtAlias(prhs[6]), "in7");
  in8 = b_emlrt_marshallIn(st, emlrtAlias(prhs[7]), "in8");
  in9 = emlrt_marshallIn(st, emlrtAlias(prhs[8]), "in9");
  in10 = b_emlrt_marshallIn(st, emlrtAlias(prhs[9]), "in10");
  in11 = emlrt_marshallIn(st, emlrtAlias(prhs[10]), "in11");
  in12 = emlrt_marshallIn(st, emlrtAlias(prhs[11]), "in12");
  in13 = emlrt_marshallIn(st, emlrtAlias(prhs[12]), "in13");
  in14 = emlrt_marshallIn(st, emlrtAlias(prhs[13]), "in14");
  in15 = emlrt_marshallIn(st, emlrtAlias(prhs[14]), "in15");
  in16 = b_emlrt_marshallIn(st, emlrtAlias(prhs[15]), "in16");
  in17 = emlrt_marshallIn(st, emlrtAlias(prhs[16]), "in17");
  in18 = b_emlrt_marshallIn(st, emlrtAlias(prhs[17]), "in18");
  in19 = emlrt_marshallIn(st, emlrtAlias(prhs[18]), "in19");
  in20 = b_emlrt_marshallIn(st, emlrtAlias(prhs[19]), "in20");
  in21 = c_emlrt_marshallIn(st, emlrtAlias(prhs[20]), "in21");
  in22 = c_emlrt_marshallIn(st, emlrtAlias(prhs[21]), "in22");
  in23 = c_emlrt_marshallIn(st, emlrtAlias(prhs[22]), "in23");
  in24 = c_emlrt_marshallIn(st, emlrtAlias(prhs[23]), "in24");
  // Invoke the target function
  jacobian_h_to_oTg1_oTg2(*in1, *in2, *in3, *in4, *in5, *in6, *in7, *in8, *in9,
                          *in10, *in11, *in12, *in13, *in14, *in15, *in16,
                          *in17, *in18, *in19, *in20, *in21, *in22, *in23,
                          *in24, *b_jacobian_h_to_oTg1_oTg2);
  // Marshall function outputs
  *plhs = b_emlrt_marshallOut(*b_jacobian_h_to_oTg1_oTg2);
}

//
// Arguments    : const mxArray * const prhs[24]
//                const mxArray **plhs
// Return Type  : void
//
void jacobian_h_to_x_state_not_ext_api(const mxArray *const prhs[24],
                                       const mxArray **plhs)
{
  emlrtStack st{
      nullptr, // site
      nullptr, // tls
      nullptr  // prev
  };
  real_T(*b_jacobian_h_to_x_state_not_ext)[324];
  real_T(*in21)[6];
  real_T(*in22)[6];
  real_T(*in23)[6];
  real_T(*in24)[6];
  real_T(*in10)[4];
  real_T(*in16)[4];
  real_T(*in18)[4];
  real_T(*in2)[4];
  real_T(*in20)[4];
  real_T(*in6)[4];
  real_T(*in8)[4];
  real_T(*in1)[3];
  real_T(*in11)[3];
  real_T(*in12)[3];
  real_T(*in13)[3];
  real_T(*in14)[3];
  real_T(*in15)[3];
  real_T(*in17)[3];
  real_T(*in19)[3];
  real_T(*in3)[3];
  real_T(*in4)[3];
  real_T(*in5)[3];
  real_T(*in7)[3];
  real_T(*in9)[3];
  st.tls = emlrtRootTLSGlobal;
  b_jacobian_h_to_x_state_not_ext =
      (real_T(*)[324])mxMalloc(sizeof(real_T[324]));
  // Marshall function inputs
  in1 = emlrt_marshallIn(st, emlrtAlias(prhs[0]), "in1");
  in2 = b_emlrt_marshallIn(st, emlrtAlias(prhs[1]), "in2");
  in3 = emlrt_marshallIn(st, emlrtAlias(prhs[2]), "in3");
  in4 = emlrt_marshallIn(st, emlrtAlias(prhs[3]), "in4");
  in5 = emlrt_marshallIn(st, emlrtAlias(prhs[4]), "in5");
  in6 = b_emlrt_marshallIn(st, emlrtAlias(prhs[5]), "in6");
  in7 = emlrt_marshallIn(st, emlrtAlias(prhs[6]), "in7");
  in8 = b_emlrt_marshallIn(st, emlrtAlias(prhs[7]), "in8");
  in9 = emlrt_marshallIn(st, emlrtAlias(prhs[8]), "in9");
  in10 = b_emlrt_marshallIn(st, emlrtAlias(prhs[9]), "in10");
  in11 = emlrt_marshallIn(st, emlrtAlias(prhs[10]), "in11");
  in12 = emlrt_marshallIn(st, emlrtAlias(prhs[11]), "in12");
  in13 = emlrt_marshallIn(st, emlrtAlias(prhs[12]), "in13");
  in14 = emlrt_marshallIn(st, emlrtAlias(prhs[13]), "in14");
  in15 = emlrt_marshallIn(st, emlrtAlias(prhs[14]), "in15");
  in16 = b_emlrt_marshallIn(st, emlrtAlias(prhs[15]), "in16");
  in17 = emlrt_marshallIn(st, emlrtAlias(prhs[16]), "in17");
  in18 = b_emlrt_marshallIn(st, emlrtAlias(prhs[17]), "in18");
  in19 = emlrt_marshallIn(st, emlrtAlias(prhs[18]), "in19");
  in20 = b_emlrt_marshallIn(st, emlrtAlias(prhs[19]), "in20");
  in21 = c_emlrt_marshallIn(st, emlrtAlias(prhs[20]), "in21");
  in22 = c_emlrt_marshallIn(st, emlrtAlias(prhs[21]), "in22");
  in23 = c_emlrt_marshallIn(st, emlrtAlias(prhs[22]), "in23");
  in24 = c_emlrt_marshallIn(st, emlrtAlias(prhs[23]), "in24");
  // Invoke the target function
  jacobian_h_to_x_state_not_ext(*in1, *in2, *in3, *in4, *in5, *in6, *in7, *in8,
                                *in9, *in10, *in11, *in12, *in13, *in14, *in15,
                                *in16, *in17, *in18, *in19, *in20, *in21, *in22,
                                *in23, *in24, *b_jacobian_h_to_x_state_not_ext);
  // Marshall function outputs
  *plhs = c_emlrt_marshallOut(*b_jacobian_h_to_x_state_not_ext);
}

//
// File trailer for _coder_jacobian_WRh_to_oTg1_oTg2_api.cpp
//
// [EOF]
//
