//
// Academic License - for use in teaching, academic research, and meeting
// course requirements at degree granting institutions only.  Not for
// government, commercial, or other organizational use.
// File: jacobian_WRh_to_oTg1_oTg2.cpp
//
// MATLAB Coder version            : 23.2
// C/C++ source code generated on  : 11-Apr-2025 10:25:57
//

// Include Files
#include "jacobian_WRh_to_oTg1_oTg2.h"
#include "rt_nonfinite.h"
// #include "rt_nonfinite.cpp"


// Function Definitions
//
// jacobian_WRh_to_oTg1_oTg2
//     jacobian_WRh_to_oTg1_oTg2 =
//     jacobian_WRh_to_oTg1_oTg2(IN1,IN2,IN3,IN4,IN5,IN6,IN7,IN8,IN9,IN10,IN11,IN12,IN13,IN14,IN15,IN16,IN17,IN18,IN19,IN20,IN21,IN22,IN23,IN24)
//
// Arguments    : const double in1[3]
//                const double in2[4]
//                const double in3[3]
//                const double in4[3]
//                const double in5[3]
//                const double in6[4]
//                const double in7[3]
//                const double in8[4]
//                const double in9[3]
//                const double in10[4]
//                const double in11[3]
//                const double in12[3]
//                const double in13[3]
//                const double in14[3]
//                const double in15[3]
//                const double in16[4]
//                const double in17[3]
//                const double in18[4]
//                const double in19[3]
//                const double in20[4]
//                const double in21[6]
//                const double in22[6]
//                const double in23[6]
//                const double in24[6]
//                double b_jacobian_WRh_to_oTg1_oTg2[84]
// Return Type  : void
//
void jacobian_WRh_to_oTg1_oTg2(
    const double in1[3], const double in2[4], const double in3[3],
    const double in4[3], const double in5[3], const double in6[4],
    const double in7[3], const double in8[4], const double in9[3],
    const double in10[4], const double in11[3], const double in12[3],
    const double in13[3], const double in14[3], const double in15[3],
    const double in16[4], const double in17[3], const double in18[4],
    const double in19[3], const double in20[4], const double in21[6],
    const double in22[6], const double in23[6], const double in24[6],
    double b_jacobian_WRh_to_oTg1_oTg2[84])
{
  double b_ct_idx_119_tmp;
  double b_ct_idx_132_tmp;
  double b_ct_idx_133_tmp;
  double b_ct_idx_180_tmp;
  double b_ct_idx_181_tmp;
  double b_ct_idx_182_tmp;
  double b_ct_idx_183_tmp;
  double b_ct_idx_187_tmp;
  double b_ct_idx_189_tmp;
  double b_ct_idx_245_tmp;
  double b_ct_idx_246_tmp;
  double b_ct_idx_249;
  double b_ct_idx_253;
  double b_ct_idx_254;
  double b_ct_idx_255;
  double b_ct_idx_256;
  double b_ct_idx_257_tmp;
  double b_ct_idx_258_tmp;
  double b_ct_idx_259_tmp;
  double b_ct_idx_398_tmp;
  double b_ct_idx_400_tmp;
  double b_ct_idx_433_tmp;
  double b_ct_idx_435_tmp;
  double b_ct_idx_436_tmp;
  double b_ct_idx_88;
  double b_ct_idx_89;
  double b_ct_idx_90;
  double b_ct_idx_91;
  double b_ct_idx_92;
  double b_ct_idx_94;
  double b_ct_idx_95;
  double b_ct_idx_96;
  double b_ct_idx_97;
  double b_ct_idx_98;
  double b_ct_idx_99;
  double b_t2721_tmp;
  double c_ct_idx_132_tmp;
  double c_ct_idx_133_tmp;
  double c_ct_idx_181_tmp;
  double c_ct_idx_254;
  double c_ct_idx_435_tmp;
  double ct_idx_100;
  double ct_idx_101;
  double ct_idx_102;
  double ct_idx_103;
  double ct_idx_104;
  double ct_idx_105;
  double ct_idx_106;
  double ct_idx_107;
  double ct_idx_108;
  double ct_idx_109;
  double ct_idx_109_tmp;
  double ct_idx_111_tmp;
  double ct_idx_111_tmp_tmp;
  double ct_idx_117;
  double ct_idx_118;
  double ct_idx_119;
  double ct_idx_119_tmp;
  double ct_idx_122;
  double ct_idx_130_tmp;
  double ct_idx_130_tmp_tmp;
  double ct_idx_131;
  double ct_idx_132;
  double ct_idx_132_tmp;
  double ct_idx_133;
  double ct_idx_133_tmp;
  double ct_idx_134;
  double ct_idx_135;
  double ct_idx_136;
  double ct_idx_138;
  double ct_idx_149;
  double ct_idx_149_tmp;
  double ct_idx_150;
  double ct_idx_150_tmp;
  double ct_idx_160;
  double ct_idx_163;
  double ct_idx_164;
  double ct_idx_165;
  double ct_idx_166;
  double ct_idx_168;
  double ct_idx_169;
  double ct_idx_174;
  double ct_idx_180_tmp;
  double ct_idx_181_tmp;
  double ct_idx_182_tmp;
  double ct_idx_183_tmp;
  double ct_idx_187_tmp;
  double ct_idx_188;
  double ct_idx_189;
  double ct_idx_189_tmp;
  double ct_idx_190;
  double ct_idx_192;
  double ct_idx_193;
  double ct_idx_203;
  double ct_idx_204;
  double ct_idx_204_tmp;
  double ct_idx_205;
  double ct_idx_206;
  double ct_idx_206_tmp;
  double ct_idx_207;
  double ct_idx_208;
  double ct_idx_208_tmp;
  double ct_idx_209;
  double ct_idx_210;
  double ct_idx_210_tmp;
  double ct_idx_211;
  double ct_idx_211_tmp;
  double ct_idx_212;
  double ct_idx_213;
  double ct_idx_213_tmp;
  double ct_idx_222;
  double ct_idx_223;
  double ct_idx_224;
  double ct_idx_225;
  double ct_idx_226;
  double ct_idx_245_tmp;
  double ct_idx_246_tmp;
  double ct_idx_247;
  double ct_idx_248;
  double ct_idx_249;
  double ct_idx_250;
  double ct_idx_251;
  double ct_idx_252;
  double ct_idx_253;
  double ct_idx_254;
  double ct_idx_255;
  double ct_idx_256;
  double ct_idx_257;
  double ct_idx_257_tmp;
  double ct_idx_258;
  double ct_idx_258_tmp;
  double ct_idx_259;
  double ct_idx_259_tmp;
  double ct_idx_260;
  double ct_idx_261;
  double ct_idx_271;
  double ct_idx_272;
  double ct_idx_273;
  double ct_idx_281;
  double ct_idx_282;
  double ct_idx_283;
  double ct_idx_294;
  double ct_idx_295;
  double ct_idx_296;
  double ct_idx_300;
  double ct_idx_301;
  double ct_idx_302;
  double ct_idx_304;
  double ct_idx_305;
  double ct_idx_306;
  double ct_idx_375;
  double ct_idx_376;
  double ct_idx_377;
  double ct_idx_378;
  double ct_idx_379;
  double ct_idx_380;
  double ct_idx_381_tmp;
  double ct_idx_390_tmp;
  double ct_idx_391_tmp;
  double ct_idx_394;
  double ct_idx_398_tmp;
  double ct_idx_39_tmp;
  double ct_idx_400_tmp;
  double ct_idx_419_tmp;
  double ct_idx_431_tmp;
  double ct_idx_433_tmp;
  double ct_idx_435_tmp;
  double ct_idx_436_tmp;
  double ct_idx_440_tmp;
  double ct_idx_444_tmp;
  double ct_idx_447_tmp;
  double ct_idx_449_tmp;
  double ct_idx_462;
  double ct_idx_463;
  double ct_idx_513;
  double ct_idx_513_tmp;
  double ct_idx_529_tmp;
  double ct_idx_540_tmp;
  double ct_idx_55_tmp;
  double ct_idx_59_tmp;
  double ct_idx_679_tmp;
  double ct_idx_690_tmp;
  double ct_idx_720;
  double ct_idx_721;
  double ct_idx_722;
  double ct_idx_765_tmp;
  double ct_idx_784_tmp;
  double ct_idx_79_tmp;
  double ct_idx_88;
  double ct_idx_89;
  double ct_idx_90;
  double ct_idx_91;
  double ct_idx_92;
  double ct_idx_92_tmp;
  double ct_idx_93;
  double ct_idx_93_tmp;
  double ct_idx_93_tmp_tmp;
  double ct_idx_94;
  double ct_idx_95;
  double ct_idx_95_tmp;
  double ct_idx_96;
  double ct_idx_960;
  double ct_idx_97;
  double ct_idx_97_tmp;
  double ct_idx_98;
  double ct_idx_98_tmp;
  double ct_idx_99;
  double t100;
  double t101;
  double t135;
  double t136;
  double t137;
  double t138;
  double t139;
  double t141;
  double t144;
  double t145;
  double t146;
  double t147;
  double t148;
  double t149;
  double t1556;
  double t1557;
  double t1558;
  double t1691;
  double t1692;
  double t1693;
  double t1697;
  double t1698;
  double t1699;
  double t1703;
  double t1704;
  double t1705;
  double t1706;
  double t1707;
  double t1708;
  double t1709;
  double t1710;
  double t1711;
  double t1717;
  double t1718;
  double t1719;
  double t1723;
  double t1730;
  double t1731;
  double t1735;
  double t1770;
  double t1771;
  double t1772;
  double t1773;
  double t1774;
  double t1775;
  double t1776;
  double t1777;
  double t1778;
  double t1779;
  double t1780;
  double t1781;
  double t1785;
  double t1799;
  double t1899;
  double t1900;
  double t1901;
  double t2;
  double t2228;
  double t2230;
  double t2247;
  double t2247_tmp;
  double t2248;
  double t2248_tmp;
  double t2249;
  double t2249_tmp;
  double t2419;
  double t2420;
  double t2421;
  double t2483;
  double t2484;
  double t2485;
  double t2562;
  double t2598;
  double t2599;
  double t2600;
  double t2713;
  double t2715;
  double t2721;
  double t2721_tmp;
  double t2723;
  double t2723_tmp;
  double t274;
  double t275;
  double t276;
  double t277;
  double t278;
  double t279;
  double t289;
  double t290;
  double t291;
  double t292;
  double t293;
  double t294;
  double t295;
  double t296;
  double t297;
  double t298;
  double t299;
  double t3;
  double t300;
  double t4;
  double t5;
  double t523;
  double t524;
  double t525;
  double t526;
  double t527;
  double t528;
  double t57;
  double t59;
  double t6;
  double t62;
  double t7;
  double t78;
  double t79;
  double t80;
  double t81;
  double t82;
  double t83;
  double t84;
  double t85;
  double t86;
  double t87;
  double t88;
  double t89;
  double t90;
  double t91;
  double t92;
  double t93;
  double t94;
  double t95;
  double t96;
  double t97;
  double t98;
  double t99;
  //     This function was generated by the Symbolic Math Toolbox version 23.2.
  //     10-Apr-2025 15:54:52
  t2 = in10[0] * in10[1];
  t3 = in10[0] * in10[2];
  t4 = in10[0] * in10[3];
  t5 = in10[1] * in10[2];
  t6 = in10[1] * in10[3];
  t7 = in10[2] * in10[3];
  t57 = in10[1] * in10[1];
  t59 = in10[2] * in10[2];
  t62 = in10[3] * in10[3];
  t78 = in16[0] * in16[1] * 2.0;
  t79 = in16[0] * in16[2] * 2.0;
  t80 = in16[0] * in16[3] * 2.0;
  t81 = in16[1] * in16[2] * 2.0;
  t82 = in16[1] * in16[3] * 2.0;
  t83 = in16[2] * in16[3] * 2.0;
  t84 = in2[0] * in2[1] * 2.0;
  t85 = in2[0] * in2[2] * 2.0;
  t86 = in2[0] * in2[3] * 2.0;
  t87 = in2[1] * in2[2] * 2.0;
  t88 = in2[1] * in2[3] * 2.0;
  t89 = in2[2] * in2[3] * 2.0;
  t90 = in18[0] * in18[1] * 2.0;
  t91 = in18[0] * in18[2] * 2.0;
  t92 = in18[0] * in18[3] * 2.0;
  t93 = in18[1] * in18[2] * 2.0;
  t94 = in18[1] * in18[3] * 2.0;
  t95 = in18[2] * in18[3] * 2.0;
  t96 = in20[0] * in20[1] * 2.0;
  t97 = in20[0] * in20[2] * 2.0;
  t98 = in20[0] * in20[3] * 2.0;
  t99 = in20[1] * in20[2] * 2.0;
  t100 = in20[1] * in20[3] * 2.0;
  t101 = in20[2] * in20[3] * 2.0;
  t135 = t57 * 2.0;
  t136 = t59 * 2.0;
  t137 = t57 * 4.0;
  t138 = t62 * 2.0;
  t139 = t59 * 4.0;
  t141 = t62 * 4.0;
  t144 = in16[1] * in16[1] * 2.0;
  t145 = in16[2] * in16[2] * 2.0;
  t146 = in16[3] * in16[3] * 2.0;
  t147 = in2[1] * in2[1] * 2.0;
  t148 = in2[2] * in2[2] * 2.0;
  t149 = in2[3] * in2[3] * 2.0;
  t274 = t2 * t2 * 4.0;
  t275 = t3 * t3 * 4.0;
  t276 = t4 * t4 * 4.0;
  t1709 = t5 * t5;
  t277 = t1709 * 4.0;
  t1710 = t6 * t6;
  t278 = t1710 * 4.0;
  t1711 = t7 * t7;
  t279 = t1711 * 4.0;
  t289 = t78 + t83;
  t290 = t79 + t82;
  t291 = t80 + t81;
  t292 = t84 + t89;
  t293 = t85 + t88;
  t294 = t86 + t87;
  t295 = t90 + t95;
  t296 = t91 + t94;
  t297 = t92 + t93;
  t298 = t96 + t101;
  t299 = t97 + t100;
  t300 = t98 + t99;
  ct_idx_117 = t57 * t57 * 4.0;
  ct_idx_118 = t59 * t59 * 4.0;
  ct_idx_119 = t62 * t62 * 4.0;
  ct_idx_133 = in18[1] * in18[1] * 2.0;
  ct_idx_134 = in18[2] * in18[2] * 2.0;
  ct_idx_135 = in18[3] * in18[3] * 2.0;
  ct_idx_136 = in20[1] * in20[1] * 2.0;
  t1785 = in20[2] * in20[2] * 2.0;
  ct_idx_138 = in20[3] * in20[3] * 2.0;
  ct_idx_160 = t5 * t136;
  t1556 = t5 * t135;
  t1558 = t6 * t138;
  ct_idx_163 = t6 * t135;
  ct_idx_164 = t7 * t138;
  ct_idx_165 = t7 * t136;
  ct_idx_166 = in9[0] * t137;
  ct_idx_168 = in9[1] * t139;
  ct_idx_169 = in9[2] * t141;
  ct_idx_188 = t2 * t3 * 2.0;
  ct_idx_189 = t2 * t4 * 2.0;
  ct_idx_190 = t3 * t4 * 2.0;
  t1557 = t5 * t138;
  ct_idx_192 = t5 * t7 * 2.0;
  ct_idx_193 = t5 * t6 * 2.0;
  ct_idx_249 = t78 - t83;
  ct_idx_250 = t79 - t82;
  ct_idx_251 = t80 - t81;
  ct_idx_252 = t84 - t89;
  ct_idx_253 = t85 - t88;
  ct_idx_254 = t86 - t87;
  ct_idx_256 = t90 - t95;
  ct_idx_257 = t91 - t94;
  ct_idx_258 = t92 - t93;
  ct_idx_259 = t96 - t101;
  ct_idx_260 = t97 - t100;
  ct_idx_261 = t98 - t99;
  ct_idx_271 = in4[0] * t294;
  ct_idx_272 = in4[1] * t292;
  ct_idx_273 = in4[2] * t293;
  ct_idx_281 = in3[0] * t294;
  ct_idx_282 = in3[1] * t292;
  ct_idx_283 = in3[2] * t293;
  ct_idx_300 = (t144 + t145) - 1.0;
  ct_idx_301 = (t144 + t146) - 1.0;
  ct_idx_302 = (t145 + t146) - 1.0;
  ct_idx_304 = (t147 + t148) - 1.0;
  ct_idx_305 = (t147 + t149) - 1.0;
  ct_idx_306 = (t148 + t149) - 1.0;
  ct_idx_394 = ((in16[0] * in2[0] + in16[1] * in2[1]) + in16[2] * in2[2]) +
               in16[3] * in2[3];
  t523 = (ct_idx_133 + ct_idx_134) - 1.0;
  t524 = (ct_idx_133 + ct_idx_135) - 1.0;
  t525 = (ct_idx_134 + ct_idx_135) - 1.0;
  t526 = (ct_idx_136 + t1785) - 1.0;
  t527 = (ct_idx_136 + ct_idx_138) - 1.0;
  t528 = (t1785 + ct_idx_138) - 1.0;
  ct_idx_79_tmp = in24[1] * in20[1];
  ct_idx_88 = in18[0] * in17[0] * 2.0;
  ct_idx_89 = in18[0] * in17[1] * 2.0;
  t78 = in17[0] * in18[1];
  ct_idx_90 = t78 * 2.0;
  ct_idx_91 = in18[0] * in17[2] * 2.0;
  ct_idx_92_tmp = in18[1] * in17[1];
  ct_idx_92 = ct_idx_92_tmp * 2.0;
  ct_idx_93_tmp_tmp = in17[0] * in18[2];
  ct_idx_93_tmp = ct_idx_93_tmp_tmp * 2.0;
  ct_idx_94 = t78 * 4.0;
  ct_idx_95_tmp = in18[1] * in17[2];
  ct_idx_95 = ct_idx_95_tmp * 2.0;
  t78 = in17[1] * in18[2];
  ct_idx_96 = t78 * 2.0;
  ct_idx_97_tmp = in18[2] * in17[2];
  ct_idx_97 = ct_idx_97_tmp * 2.0;
  ct_idx_98_tmp = in17[1] * in18[3];
  ct_idx_98 = ct_idx_98_tmp * 2.0;
  ct_idx_99 = t78 * 4.0;
  ct_idx_100 = ((in2[2] * in16[3] + in16[0] * in2[1]) - in2[0] * in16[1]) -
               in16[2] * in2[3];
  ct_idx_101 = ((in16[0] * in2[2] + in16[1] * in2[3]) - in2[0] * in16[2]) -
               in2[1] * in16[3];
  t78 = in17[2] * in18[3];
  ct_idx_102 = t78 * 2.0;
  ct_idx_103 = ((in16[0] * in2[3] + in2[1] * in16[2]) - in16[1] * in2[2]) -
               in2[0] * in16[3];
  ct_idx_104 = t78 * 4.0;
  ct_idx_105 = in20[0] * in19[0] * 2.0;
  ct_idx_106 = in20[0] * in19[1] * 2.0;
  t78 = in19[0] * in20[1];
  ct_idx_107 = t78 * 2.0;
  ct_idx_108 = in20[0] * in19[2] * 2.0;
  ct_idx_109_tmp = in20[1] * in19[1];
  ct_idx_109 = ct_idx_109_tmp * 2.0;
  ct_idx_111_tmp_tmp = in19[0] * in20[2];
  ct_idx_111_tmp = ct_idx_111_tmp_tmp * 2.0;
  ct_idx_122 = t78 * 4.0;
  ct_idx_130_tmp_tmp = in20[1] * in19[2];
  ct_idx_130_tmp = ct_idx_130_tmp_tmp * 2.0;
  t78 = in19[1] * in20[2];
  ct_idx_131 = t78 * 2.0;
  ct_idx_132_tmp = in20[2] * in19[2];
  ct_idx_132 = ct_idx_132_tmp * 2.0;
  ct_idx_133_tmp = in19[1] * in20[3];
  ct_idx_133 = ct_idx_133_tmp * 2.0;
  ct_idx_134 = t78 * 4.0;
  t78 = in19[2] * in20[3];
  ct_idx_135 = t78 * 2.0;
  ct_idx_136 = t78 * 4.0;
  ct_idx_149_tmp = in17[0] * in18[3];
  ct_idx_149 = -(ct_idx_149_tmp * 2.0);
  ct_idx_150_tmp = in19[0] * in20[3];
  ct_idx_150 = -(ct_idx_150_tmp * 2.0);
  ct_idx_294 = in4[0] * ct_idx_253;
  ct_idx_295 = in4[1] * ct_idx_254;
  ct_idx_296 = in4[2] * ct_idx_252;
  ct_idx_419_tmp = in24[1] * in20[3];
  ct_idx_431_tmp = in20[2] * in24[3];
  ct_idx_440_tmp = in24[3] * in20[3];
  ct_idx_444_tmp = in20[1] * in24[5];
  ct_idx_447_tmp = in20[3] * in24[4];
  ct_idx_449_tmp = in20[2] * in24[5];
  ct_idx_529_tmp = in22[0] * in18[2];
  ct_idx_540_tmp = in18[1] * in22[2];
  ct_idx_679_tmp = in23[0] * in20[2];
  ct_idx_690_tmp = in20[1] * in23[2];
  t78 = ((in2[0] * in18[1] + in18[0] * in2[1]) + in2[2] * in18[3]) -
        in18[2] * in2[3];
  t83 = ((in2[0] * in18[2] + in18[0] * in2[2]) + in18[1] * in2[3]) -
        in2[1] * in18[3];
  t79 = ((in2[0] * in18[3] + in2[1] * in18[2]) + in18[0] * in2[3]) -
        in18[1] * in2[2];
  t82 = ((in2[1] * in18[1] + in2[2] * in18[2]) + in2[3] * in18[3]) -
        in2[0] * in18[0];
  t80 = ((in2[0] * in20[1] + in20[0] * in2[1]) + in2[2] * in20[3]) -
        in20[2] * in2[3];
  t81 = ((in2[0] * in20[2] + in20[0] * in2[2]) + in20[1] * in2[3]) -
        in2[1] * in20[3];
  t84 = ((in2[0] * in20[3] + in2[1] * in20[2]) + in20[0] * in2[3]) -
        in20[1] * in2[2];
  t89 = ((in2[1] * in20[1] + in2[2] * in20[2]) + in2[3] * in20[3]) -
        in2[0] * in20[0];
  ct_idx_720 = in4[0] * ct_idx_306;
  ct_idx_721 = in4[1] * ct_idx_305;
  ct_idx_722 = in4[2] * ct_idx_304;
  ct_idx_765_tmp = in22[0] * in18[3];
  ct_idx_784_tmp = in22[2] * in18[2];
  ct_idx_138 = ((((-t5 + t4) + ct_idx_160) + t1556) + ct_idx_188) + t1557;
  ct_idx_462 = ((((-t6 + t3) + t1558) + ct_idx_163) + ct_idx_189) + ct_idx_192;
  ct_idx_463 =
      ((((-t7 + t2) + ct_idx_164) + ct_idx_165) + ct_idx_190) + ct_idx_193;
  ct_idx_160 = ((((-t4 - t5) + ct_idx_160) + t1556) + ct_idx_188) + t1557;
  t147 = ((((-t3 - t6) + t1558) + ct_idx_163) + ct_idx_189) + ct_idx_192;
  t149 = ((((-t2 - t7) + ct_idx_164) + ct_idx_165) + ct_idx_190) + ct_idx_193;
  t96 = ((((((ct_idx_117 - t136) - t137) - t138) + t274) + t277) + t278) + 1.0;
  t146 = ((((((ct_idx_118 - t135) - t138) - t139) + t275) + t277) + t279) + 1.0;
  t148 = ((((((ct_idx_119 - t135) - t136) - t141) + t276) + t278) + t279) + 1.0;
  ct_idx_164 = -(in3[0] * ct_idx_253);
  t136 = -(in3[1] * ct_idx_254);
  ct_idx_190 = -(in3[2] * ct_idx_252);
  ct_idx_960 = -(in3[0] * ct_idx_306);
  t138 = -(in3[1] * ct_idx_305);
  ct_idx_165 = -(in3[2] * ct_idx_304);
  t1556 = (ct_idx_294 - ct_idx_272) + ct_idx_722;
  t1557 = (ct_idx_296 - ct_idx_271) + ct_idx_721;
  t1558 = (ct_idx_295 - ct_idx_273) + ct_idx_720;
  t1691 = (in22[0] * t292 * t297 + in22[0] * ct_idx_253 * t525) +
          in22[0] * ct_idx_257 * ct_idx_304;
  t1692 = (in22[1] * t293 * t295 + in22[1] * ct_idx_254 * t524) +
          in22[1] * ct_idx_258 * ct_idx_306;
  t1693 = (in22[2] * t294 * t296 + in22[2] * ct_idx_252 * t523) +
          in22[2] * ct_idx_256 * ct_idx_305;
  t1697 = (in24[0] * t292 * t300 + in24[0] * ct_idx_253 * t528) +
          in24[0] * ct_idx_260 * ct_idx_304;
  t1698 = (in24[1] * t293 * t298 + in24[1] * ct_idx_254 * t527) +
          in24[1] * ct_idx_261 * ct_idx_306;
  t1699 = (in24[2] * t294 * t299 + in24[2] * ct_idx_252 * t526) +
          in24[2] * ct_idx_259 * ct_idx_305;
  t1703 = (in21[0] * t292 * t297 + in21[0] * ct_idx_253 * t525) +
          in21[0] * ct_idx_257 * ct_idx_304;
  t1704 = (in21[1] * t293 * t295 + in21[1] * ct_idx_254 * t524) +
          in21[1] * ct_idx_258 * ct_idx_306;
  t1705 = (in21[2] * t294 * t296 + in21[2] * ct_idx_252 * t523) +
          in21[2] * ct_idx_256 * ct_idx_305;
  t1706 = (in23[0] * t292 * t300 + in23[0] * ct_idx_253 * t528) +
          in23[0] * ct_idx_260 * ct_idx_304;
  t1707 = (in23[1] * t293 * t298 + in23[1] * ct_idx_254 * t527) +
          in23[1] * ct_idx_261 * ct_idx_306;
  t1708 = (in23[2] * t294 * t299 + in23[2] * ct_idx_252 * t526) +
          in23[2] * ct_idx_259 * ct_idx_305;
  t1717 = ((in6[1] * ct_idx_100 + in6[0] * ct_idx_394) + in6[2] * ct_idx_101) +
          in6[3] * ct_idx_103;
  t1785 =
      ((in10[1] * ct_idx_100 - in10[0] * ct_idx_394) + in10[2] * ct_idx_101) +
      in10[3] * ct_idx_103;
  t98 =
      1.0 / ((((((((((((ct_idx_117 + ct_idx_118) + ct_idx_119) - t137) - t139) -
                    t141) +
                   t274) +
                  t275) +
                 t276) +
                t1709 * 8.0) +
               t1710 * 8.0) +
              t1711 * 8.0) +
             1.0);
  t1709 = ((in16[0] * t78 + in16[1] * t82) + in16[3] * t83) - in16[2] * t79;
  t1710 = ((in16[0] * t83 + in16[1] * t79) + in16[2] * t82) - in16[3] * t78;
  t1711 = ((in16[0] * t79 + in16[2] * t78) + in16[3] * t82) - in16[1] * t83;
  t135 = ((in16[1] * t78 + in16[2] * t83) + in16[3] * t79) - in16[0] * t82;
  t99 = ((in16[0] * t80 + in16[1] * t89) + in16[3] * t81) - in16[2] * t84;
  t144 = ((in16[0] * t81 + in16[1] * t84) + in16[2] * t89) - in16[3] * t80;
  t145 = ((in16[0] * t84 + in16[2] * t80) + in16[3] * t89) - in16[1] * t81;
  t87 = ((in16[1] * t80 + in16[2] * t81) + in16[3] * t84) - in16[0] * t89;
  t79 = in22[0] * t297;
  t1718 = (in22[0] * t294 * t525 + t79 * ct_idx_305) -
          in22[0] * ct_idx_252 * ct_idx_257;
  t82 = in22[1] * t295;
  t1719 = (in22[1] * t292 * t524 + t82 * ct_idx_304) -
          in22[1] * ct_idx_253 * ct_idx_258;
  t80 = in22[2] * t296;
  t1723 = (in22[2] * t293 * t523 + t80 * ct_idx_306) -
          in22[2] * ct_idx_254 * ct_idx_256;
  t81 = in24[0] * t300;
  t1730 = (in24[0] * t294 * t528 + t81 * ct_idx_305) -
          in24[0] * ct_idx_252 * ct_idx_260;
  t84 = in24[1] * t298;
  t1731 = (in24[1] * t292 * t527 + t84 * ct_idx_304) -
          in24[1] * ct_idx_253 * ct_idx_261;
  t89 = in24[2] * t299;
  t1735 = (in24[2] * t293 * t526 + t89 * ct_idx_306) -
          in24[2] * ct_idx_254 * ct_idx_259;
  t78 = in21[0] * t297;
  t1770 = (in21[0] * t294 * t525 + t78 * ct_idx_305) -
          in21[0] * ct_idx_252 * ct_idx_257;
  t83 = in21[1] * t295;
  t1771 = (in21[1] * t292 * t524 + t83 * ct_idx_304) -
          in21[1] * ct_idx_253 * ct_idx_258;
  t1772 = (in21[0] * t293 * ct_idx_257 + t78 * ct_idx_254) -
          in21[0] * ct_idx_306 * t525;
  t1773 = (t83 * ct_idx_252 + in21[1] * t294 * ct_idx_258) -
          in21[1] * ct_idx_305 * t524;
  t78 = in21[2] * t296;
  t1774 = (in21[2] * t292 * ct_idx_256 + t78 * ct_idx_253) -
          in21[2] * ct_idx_304 * t523;
  t1775 = (in21[2] * t293 * t523 + t78 * ct_idx_306) -
          in21[2] * ct_idx_254 * ct_idx_256;
  t78 = in23[0] * t300;
  t1776 = (in23[0] * t294 * t528 + t78 * ct_idx_305) -
          in23[0] * ct_idx_252 * ct_idx_260;
  t83 = in23[1] * t298;
  t1777 = (in23[1] * t292 * t527 + t83 * ct_idx_304) -
          in23[1] * ct_idx_253 * ct_idx_261;
  t1778 = (in23[0] * t293 * ct_idx_260 + t78 * ct_idx_254) -
          in23[0] * ct_idx_306 * t528;
  t1779 = (t83 * ct_idx_252 + in23[1] * t294 * ct_idx_261) -
          in23[1] * ct_idx_305 * t527;
  t78 = in23[2] * t299;
  t1780 = (in23[2] * t292 * ct_idx_259 + t78 * ct_idx_253) -
          in23[2] * ct_idx_304 * t526;
  t1781 = (in23[2] * t293 * t526 + t78 * ct_idx_306) -
          in23[2] * ct_idx_254 * ct_idx_259;
  t85 = ((in10[0] * ct_idx_100 + in10[1] * ct_idx_394) + in10[2] * ct_idx_103) -
        in10[3] * ct_idx_101;
  t88 = ((in10[0] * ct_idx_101 + in10[2] * ct_idx_394) + in10[3] * ct_idx_100) -
        in10[1] * ct_idx_103;
  t86 = ((in10[0] * ct_idx_103 + in10[3] * ct_idx_394) + in10[1] * ct_idx_101) -
        in10[2] * ct_idx_100;
  t1799 = ((in6[2] * ct_idx_103 + in6[1] * ct_idx_394) - in6[0] * ct_idx_100) -
          in6[3] * ct_idx_101;
  ct_idx_188 = in15[0] - in1[0];
  t1899 = (((((ct_idx_188 + in5[2] * t290) + in17[1] * ct_idx_254) -
             in17[2] * t293) -
            in5[1] * ct_idx_251) +
           in17[0] * ct_idx_306) -
          in5[0] * ct_idx_302;
  ct_idx_163 = in15[1] - in1[1];
  t1900 = (((((ct_idx_163 + in5[0] * t291) + in17[2] * ct_idx_252) -
             in17[0] * t294) -
            in5[2] * ct_idx_249) +
           in17[1] * ct_idx_305) -
          in5[1] * ct_idx_301;
  ct_idx_189 = in15[2] - in1[2];
  t1901 = (((((ct_idx_189 + in5[1] * t289) + in17[0] * ct_idx_253) -
             in17[1] * t292) -
            in5[0] * ct_idx_250) +
           in17[2] * ct_idx_304) -
          in5[2] * ct_idx_300;
  ct_idx_39_tmp = in20[1] * in24[4];
  ct_idx_55_tmp = in23[0] * in20[3];
  ct_idx_59_tmp = in23[2] * in20[2];
  b_ct_idx_88 = in17[1] * t296 + in17[0] * ct_idx_256;
  b_ct_idx_89 = in17[0] * t295 + in17[2] * ct_idx_258;
  b_ct_idx_90 = in17[2] * t297 + in17[1] * ct_idx_257;
  b_ct_idx_91 = in19[1] * t299 + in19[0] * ct_idx_259;
  b_ct_idx_92 = in19[0] * t298 + in19[2] * ct_idx_261;
  ct_idx_93 = in19[2] * t300 + in19[1] * ct_idx_260;
  b_ct_idx_94 = in17[2] * t296 + in17[0] * t523;
  b_ct_idx_95 = in17[0] * t297 + in17[1] * t525;
  b_ct_idx_96 = in17[1] * t295 + in17[2] * t524;
  b_ct_idx_97 = in19[2] * t299 + in19[0] * t526;
  b_ct_idx_98 = in19[0] * t300 + in19[1] * t528;
  b_ct_idx_99 = in19[1] * t298 + in19[2] * t527;
  ct_idx_119_tmp = in18[2] * in22[5];
  b_ct_idx_119_tmp = in18[1] * in22[5];
  b_ct_idx_132_tmp = in22[3] * in18[3];
  c_ct_idx_132_tmp = in18[2] * in22[3];
  b_ct_idx_133_tmp = in18[1] * in22[4];
  c_ct_idx_133_tmp = in18[3] * in22[4];
  ct_idx_183_tmp = in18[0] * in22[3];
  b_ct_idx_183_tmp = in18[1] * in22[3];
  ct_idx_187_tmp = in18[0] * in22[4];
  b_ct_idx_187_tmp = in18[2] * in22[4];
  ct_idx_189_tmp = in18[0] * in22[5];
  b_ct_idx_189_tmp = in18[3] * in22[5];
  ct_idx_247 = (in22[0] * t293 * ct_idx_257 + t79 * ct_idx_254) -
               in22[0] * ct_idx_306 * t525;
  ct_idx_248 = (t82 * ct_idx_252 + in22[1] * t294 * ct_idx_258) -
               in22[1] * ct_idx_305 * t524;
  b_ct_idx_249 = (in22[2] * t292 * ct_idx_256 + t80 * ct_idx_253) -
                 in22[2] * ct_idx_304 * t523;
  b_ct_idx_253 = (in24[0] * t293 * ct_idx_260 + t81 * ct_idx_254) -
                 in24[0] * ct_idx_306 * t528;
  b_ct_idx_254 = (t84 * ct_idx_252 + in24[1] * t294 * ct_idx_261) -
                 in24[1] * ct_idx_305 * t527;
  ct_idx_255 = (in24[2] * t292 * ct_idx_259 + t89 * ct_idx_253) -
               in24[2] * ct_idx_304 * t526;
  ct_idx_375 = t294 * t1556 + ct_idx_253 * t1557;
  ct_idx_376 = t292 * t1558 + ct_idx_254 * t1556;
  ct_idx_377 = t293 * t1557 + ct_idx_252 * t1558;
  ct_idx_378 = t292 * t1557 + ct_idx_305 * t1556;
  ct_idx_379 = t293 * t1556 + ct_idx_304 * t1558;
  ct_idx_380 = t294 * t1558 + ct_idx_306 * t1557;
  ct_idx_381_tmp =
      ((in12[1] * t289 - in12[0] * ct_idx_250) - in12[2] * ct_idx_300) + t1556;
  ct_idx_390_tmp =
      ((in12[0] * t291 - in12[2] * ct_idx_249) - in12[1] * ct_idx_301) + t1557;
  ct_idx_391_tmp =
      ((in12[2] * t290 - in12[1] * ct_idx_251) - in12[0] * ct_idx_302) + t1558;
  ct_idx_398_tmp = in21[0] * in18[3];
  b_ct_idx_398_tmp = in21[0] * in18[2];
  ct_idx_400_tmp = in21[2] * in18[2];
  b_ct_idx_400_tmp = in18[1] * in21[2];
  ct_idx_433_tmp = in21[0] * in18[0];
  b_ct_idx_433_tmp = in21[0] * in18[1];
  ct_idx_435_tmp = in21[1] * in18[3];
  b_ct_idx_435_tmp = in18[0] * in21[1];
  c_ct_idx_435_tmp = in21[1] * in18[2];
  ct_idx_436_tmp = in21[2] * in18[3];
  b_ct_idx_436_tmp = in18[0] * in21[2];
  ct_idx_513_tmp =
      ((in6[2] * ct_idx_394 - in6[0] * ct_idx_101) - in6[1] * ct_idx_103) +
      in6[3] * ct_idx_100;
  ct_idx_513 =
      ((in21[4] * ct_idx_513_tmp +
        -(c_ct_idx_133_tmp * t294 * 2.0 + b_ct_idx_133_tmp * ct_idx_252 * 2.0) *
            ct_idx_390_tmp) +
       (c_ct_idx_133_tmp * ct_idx_253 * 2.0 -
        b_ct_idx_133_tmp * ct_idx_304 * 2.0) *
           ct_idx_381_tmp) +
      (b_ct_idx_133_tmp * t293 * 2.0 + c_ct_idx_133_tmp * ct_idx_306 * 2.0) *
          ct_idx_391_tmp;
  t2230 = ((in8[1] * t85 + in8[2] * t88) + in8[3] * t86) - in8[0] * t1785;
  t90 =
      (-(ct_idx_160 * t289 * t98 * 2.0) + ct_idx_462 * ct_idx_300 * t98 * 2.0) +
      t96 * ct_idx_250 * t98;
  t101 =
      (-(ct_idx_462 * t290 * t98 * 2.0) + ct_idx_160 * ct_idx_251 * t98 * 2.0) +
      t96 * ct_idx_302 * t98;
  t94 = (-(ct_idx_138 * t291 * t98 * 2.0) + t149 * ct_idx_249 * t98 * 2.0) +
        t146 * ct_idx_301 * t98;
  t97 = (-(t149 * t290 * t98 * 2.0) + ct_idx_138 * ct_idx_302 * t98 * 2.0) +
        t146 * ct_idx_251 * t98;
  t95 = (-(ct_idx_463 * t289 * t98 * 2.0) + t147 * ct_idx_250 * t98 * 2.0) +
        t148 * ct_idx_300 * t98;
  t92 = (-(t147 * t291 * t98 * 2.0) + ct_idx_463 * ct_idx_301 * t98 * 2.0) +
        t148 * ct_idx_249 * t98;
  t93 = (ct_idx_462 * ct_idx_249 * t98 * 2.0 +
         ct_idx_160 * ct_idx_301 * t98 * 2.0) -
        t96 * t291 * t98;
  t91 = (ct_idx_138 * ct_idx_250 * t98 * 2.0 + t149 * ct_idx_300 * t98 * 2.0) -
        t146 * t289 * t98;
  t100 = (ct_idx_463 * ct_idx_251 * t98 * 2.0 + t147 * ct_idx_302 * t98 * 2.0) -
         t148 * t290 * t98;
  t84 = in9[1] * t5;
  t78 = in9[2] * t6;
  t83 = in9[2] * t2;
  t80 = in9[2] * t3;
  t81 = in9[2] * t5;
  t89 = in9[1] * t2;
  t96 = ((in7[1] * ct_idx_138 * t98 * 2.0 + in7[2] * t147 * t98 * 2.0) +
         in7[0] * t96 * t98) -
        t98 * (((((((((((((((((((in9[0] + in9[1] * t4 * 2.0) +
                                in9[0] * ct_idx_117) -
                               t80 * 2.0) -
                              t84 * 2.0) -
                             t78 * 2.0) +
                            in9[0] * t59 * -2.0) +
                           in9[0] * t57 * -4.0) +
                          in9[0] * t62 * -2.0) +
                         ct_idx_168 * t5) +
                        t84 * t137) +
                       ct_idx_169 * t6) +
                      t78 * t137) +
                     t89 * t3 * 4.0) +
                    t83 * t4 * 4.0) +
                   t84 * t141) +
                  t81 * t7 * 4.0) +
                 in9[0] * t274) +
                in9[0] * t277) +
               in9[0] * t278);
  t82 = in9[0] * t5;
  t78 = in9[2] * t7;
  t79 = in9[0] * t2;
  t81 = ((in7[2] * ct_idx_463 * t98 * 2.0 + in7[0] * ct_idx_160 * t98 * 2.0) +
         in7[1] * t146 * t98) -
        t98 * (((((((((((((((((((in9[1] + t83 * 2.0) + in9[1] * ct_idx_118) -
                               in9[0] * t4 * 2.0) -
                              t82 * 2.0) -
                             t78 * 2.0) +
                            in9[1] * t57 * -2.0) +
                           in9[1] * t62 * -2.0) +
                          in9[1] * t59 * -4.0) +
                         t82 * t139) +
                        ct_idx_166 * t5) +
                       ct_idx_169 * t7) +
                      t78 * t139) +
                     t79 * t3 * 4.0) +
                    t82 * t141) +
                   t80 * t4 * 4.0) +
                  t81 * t6 * 4.0) +
                 in9[1] * t275) +
                in9[1] * t277) +
               in9[1] * t279);
  t78 = in9[0] * t6;
  t83 = in9[1] * t7;
  t80 = ((in7[0] * ct_idx_462 * t98 * 2.0 + in7[1] * t149 * t98 * 2.0) +
         in7[2] * t148 * t98) -
        t98 * (((((((((((((((((((in9[2] + in9[0] * t3 * 2.0) +
                                in9[2] * ct_idx_119) -
                               t89 * 2.0) -
                              t78 * 2.0) -
                             t83 * 2.0) +
                            in9[2] * t57 * -2.0) +
                           in9[2] * t59 * -2.0) +
                          in9[2] * t62 * -4.0) +
                         t78 * t141) +
                        ct_idx_166 * t6) +
                       t83 * t141) +
                      ct_idx_168 * t7) +
                     t79 * t4 * 4.0) +
                    t82 * t7 * 4.0) +
                   in9[1] * t3 * t4 * 4.0) +
                  t84 * t6 * 4.0) +
                 in9[2] * t276) +
                in9[2] * t278) +
               in9[2] * t279);
  t146 = ((in10[0] * t99 + in10[1] * t87) + in10[2] * t145) - in10[3] * t144;
  t147 = ((in10[0] * t144 + in10[2] * t87) + in10[3] * t99) - in10[1] * t145;
  t148 = ((in10[0] * t145 + in10[1] * t144) + in10[3] * t87) - in10[2] * t99;
  t98 = ((in10[1] * t99 + in10[2] * t144) + in10[3] * t145) - in10[0] * t87;
  t87 = ((in8[0] * t85 + in8[1] * t1785) + in8[3] * t88) - in8[2] * t86;
  t2228 = ((in8[0] * t88 + in8[1] * t86) + in8[2] * t1785) - in8[3] * t85;
  t85 = ((in8[0] * t86 + in8[2] * t85) + in8[3] * t1785) - in8[1] * t88;
  t2247_tmp = ct_idx_252 * t1556 - ct_idx_304 * t1557;
  t2247 = (((((((ct_idx_283 + in11[1] * ct_idx_251) - in11[2] * t290) + t136) +
              in11[0] * ct_idx_302) +
             ct_idx_960) +
            in17[0] * ct_idx_375) -
           in17[1] * ct_idx_378) +
          -in17[2] * t2247_tmp;
  t2248_tmp = ct_idx_253 * t1558 - ct_idx_306 * t1556;
  t2248 = (((((((ct_idx_281 + in11[2] * ct_idx_249) - in11[0] * t291) +
               ct_idx_190) +
              in11[1] * ct_idx_301) +
             t138) +
            in17[1] * ct_idx_376) -
           in17[2] * ct_idx_379) +
          -in17[0] * t2248_tmp;
  t2249_tmp = ct_idx_254 * t1557 - ct_idx_305 * t1558;
  t2249 = (((((((ct_idx_282 + in11[0] * ct_idx_250) - in11[1] * t289) +
               ct_idx_164) +
              in11[2] * ct_idx_300) +
             ct_idx_165) +
            in17[2] * ct_idx_377) -
           in17[0] * ct_idx_380) +
          -in17[1] * t2249_tmp;
  t2419 = ((((ct_idx_272 - ct_idx_294) - ct_idx_722) + in14[0] * t90) +
           in14[2] * t95) +
          in14[1] * t91;
  t2420 = ((((ct_idx_271 - ct_idx_296) - ct_idx_721) + in14[1] * t94) +
           in14[2] * t92) +
          in14[0] * t93;
  t2421 = ((((ct_idx_273 - ct_idx_295) - ct_idx_720) + in14[0] * t101) +
           in14[1] * t97) +
          in14[2] * t100;
  t2483 =
      (((((((ct_idx_282 + ct_idx_164) + ct_idx_165) + in19[2] * ct_idx_377) -
          in19[0] * ct_idx_380) +
         -in19[1] * t2249_tmp) +
        in13[0] * t90) +
       in13[2] * t95) +
      in13[1] * t91;
  t2484 = (((((((ct_idx_281 + ct_idx_190) + t138) + in19[1] * ct_idx_376) -
              in19[2] * ct_idx_379) +
             -in19[0] * t2248_tmp) +
            in13[1] * t94) +
           in13[2] * t92) +
          in13[0] * t93;
  t2485 = (((((((ct_idx_283 + t136) + ct_idx_960) + in19[0] * ct_idx_375) -
              in19[1] * ct_idx_378) +
             -in19[2] * t2247_tmp) +
            in13[0] * t101) +
           in13[1] * t97) +
          in13[2] * t100;
  t78 = in22[3] * t297;
  t83 = ((-((in22[3] * t292 * t297 + in22[3] * ct_idx_253 * t525) +
            in22[3] * ct_idx_257 * ct_idx_304) *
              ct_idx_381_tmp +
          ((in22[3] * t294 * t525 + t78 * ct_idx_305) -
           in22[3] * ct_idx_252 * ct_idx_257) *
              ct_idx_390_tmp) +
         ((in22[3] * t293 * ct_idx_257 + t78 * ct_idx_254) -
          in22[3] * ct_idx_306 * t525) *
             ct_idx_391_tmp) +
        in21[3] * (((in6[0] * t1709 + in6[3] * t1710) - in6[1] * t135) -
                   in6[2] * t1711);
  t78 = in22[4] * t295;
  t82 = ((-((in22[4] * t293 * t295 + in22[4] * ct_idx_254 * t524) +
            in22[4] * ct_idx_258 * ct_idx_306) *
              ct_idx_391_tmp +
          ((in22[4] * t292 * t524 + t78 * ct_idx_304) -
           in22[4] * ct_idx_253 * ct_idx_258) *
              ct_idx_381_tmp) +
         ((t78 * ct_idx_252 + in22[4] * t294 * ct_idx_258) -
          in22[4] * ct_idx_305 * t524) *
             ct_idx_390_tmp) +
        in21[4] * (((in6[0] * t1710 + in6[1] * t1711) - in6[2] * t135) -
                   in6[3] * t1709);
  t78 = in22[5] * t296;
  t79 = ((-((in22[5] * t294 * t296 + in22[5] * ct_idx_252 * t523) +
            in22[5] * ct_idx_256 * ct_idx_305) *
              ct_idx_390_tmp +
          ((in22[5] * t292 * ct_idx_256 + t78 * ct_idx_253) -
           in22[5] * ct_idx_304 * t523) *
              ct_idx_381_tmp) +
         ((in22[5] * t293 * t523 + t78 * ct_idx_306) -
          in22[5] * ct_idx_254 * ct_idx_256) *
             ct_idx_391_tmp) +
        in21[5] * (((in6[0] * t1711 + in6[2] * t1709) - in6[1] * t1710) -
                   in6[3] * t135);
  t84 = in24[4] * t298;
  t89 = in24[5] * t299;
  t2598 = (((((ct_idx_188 + in19[1] * ct_idx_254) - in19[2] * t293) +
             in19[0] * ct_idx_306) +
            t290 * t80) -
           ct_idx_251 * t81) -
          ct_idx_302 * t96;
  t2599 = (((((ct_idx_163 + in19[2] * ct_idx_252) - in19[0] * t294) +
             in19[1] * ct_idx_305) +
            t291 * t96) -
           ct_idx_249 * t80) -
          ct_idx_301 * t81;
  t2600 = (((((ct_idx_189 + in19[0] * ct_idx_253) - in19[1] * t292) +
             in19[2] * ct_idx_304) +
            t289 * t81) -
           ct_idx_250 * t96) -
          ct_idx_300 * t80;
  t2562 = ((((t1703 * t1901 - t1770 * t1900) - t1772 * t1899) +
            ct_idx_247 * t2247) +
           t1718 * t2248) -
          t1691 * t2249;
  t2721_tmp = ((((t1698 * t2485 - t1731 * t2483) - b_ct_idx_254 * t2484) -
                t1707 * t2598) +
               t1777 * t2600) +
              t1779 * t2599;
  b_t2721_tmp = in20[1] * t2721_tmp;
  t2721 = b_t2721_tmp * -2.0;
  t2723_tmp = in20[3] * t2721_tmp;
  t2723 = t2723_tmp * -2.0;
  t78 = in24[3] * t300;
  t80 = ((in23[3] * (((in8[0] * t146 + in8[1] * t98) + in8[3] * t147) -
                     in8[2] * t148) +
          ((in24[3] * t292 * t300 + in24[3] * ct_idx_253 * t528) +
           in24[3] * ct_idx_260 * ct_idx_304) *
              t2419) -
         ((in24[3] * t294 * t528 + t78 * ct_idx_305) -
          in24[3] * ct_idx_252 * ct_idx_260) *
             t2420) -
        ((in24[3] * t293 * ct_idx_260 + t78 * ct_idx_254) -
         in24[3] * ct_idx_306 * t528) *
            t2421;
  t2713 = ((((t1697 * t2483 - t1730 * t2484) - b_ct_idx_253 * t2485) -
            t1706 * t2600) +
           t1776 * t2599) +
          t1778 * t2598;
  t2715 = ((((t1699 * t2484 - ct_idx_255 * t2483) - t1735 * t2485) +
            t1780 * t2600) -
           t1708 * t2599) +
          t1781 * t2598;
  ct_idx_174 =
      ((in21[3] * t1799 +
        -(c_ct_idx_132_tmp * t293 * 2.0 + b_ct_idx_132_tmp * ct_idx_254 * 2.0) *
            ct_idx_391_tmp) +
       (b_ct_idx_132_tmp * t292 * 2.0 + c_ct_idx_132_tmp * ct_idx_304 * 2.0) *
           ct_idx_381_tmp) +
      (c_ct_idx_132_tmp * ct_idx_252 * 2.0 -
       b_ct_idx_132_tmp * ct_idx_305 * 2.0) *
          ct_idx_390_tmp;
  ct_idx_180_tmp = in22[0] * in18[1];
  b_ct_idx_180_tmp = in22[0] * in18[0];
  ct_idx_181_tmp = in18[0] * in22[1];
  b_ct_idx_181_tmp = in22[1] * in18[2];
  c_ct_idx_181_tmp = in22[1] * in18[3];
  ct_idx_182_tmp = in22[2] * in18[3];
  b_ct_idx_182_tmp = in18[0] * in22[2];
  ct_idx_192 =
      ((in21[4] * t1799 + -((b_ct_idx_187_tmp * t293 * 2.0 +
                             c_ct_idx_133_tmp * ct_idx_254 * 4.0) +
                            ct_idx_187_tmp * ct_idx_306 * 2.0) *
                              ct_idx_391_tmp) +
       ((c_ct_idx_133_tmp * t292 * 4.0 - ct_idx_187_tmp * ct_idx_253 * 2.0) +
        b_ct_idx_187_tmp * ct_idx_304 * 2.0) *
           ct_idx_381_tmp) +
      ((ct_idx_187_tmp * t294 * 2.0 + b_ct_idx_187_tmp * ct_idx_252 * 2.0) -
       c_ct_idx_133_tmp * ct_idx_305 * 4.0) *
          ct_idx_390_tmp;
  ct_idx_193 =
      ((in21[5] * t1799 +
        -((ct_idx_119_tmp * t293 * 4.0 + b_ct_idx_189_tmp * ct_idx_254 * 2.0) +
          ct_idx_189_tmp * ct_idx_306 * 2.0) *
            ct_idx_391_tmp) +
       ((b_ct_idx_189_tmp * t292 * 2.0 - ct_idx_189_tmp * ct_idx_253 * 2.0) +
        ct_idx_119_tmp * ct_idx_304 * 4.0) *
           ct_idx_381_tmp) +
      ((ct_idx_189_tmp * t294 * 2.0 + ct_idx_119_tmp * ct_idx_252 * 4.0) -
       b_ct_idx_189_tmp * ct_idx_305 * 2.0) *
          ct_idx_390_tmp;
  ct_idx_203 = in18[0] * t83 * 2.0;
  ct_idx_204_tmp = in18[2] * t83;
  ct_idx_204 = ct_idx_204_tmp * 2.0;
  ct_idx_205 = in18[0] * t82 * 2.0;
  ct_idx_206_tmp = in18[3] * t82;
  ct_idx_206 = ct_idx_206_tmp * 2.0;
  ct_idx_207 = in18[0] * t79 * 2.0;
  ct_idx_208_tmp = in18[1] * t79;
  ct_idx_208 = ct_idx_208_tmp * 2.0;
  ct_idx_209 = -(in18[1] * t83 * 2.0);
  ct_idx_210_tmp = in18[3] * t83;
  ct_idx_210 = -(ct_idx_210_tmp * 2.0);
  ct_idx_211_tmp = in18[1] * t82;
  ct_idx_211 = -(ct_idx_211_tmp * 2.0);
  ct_idx_212 = -(in18[2] * t82 * 2.0);
  ct_idx_213_tmp = in18[2] * t79;
  ct_idx_213 = -(ct_idx_213_tmp * 2.0);
  t291 = -(in18[3] * t79 * 2.0);
  t81 = ((((t292 * t1703 + ct_idx_254 * t1772) + ct_idx_305 * t1770) -
          t1718 * ct_idx_376) +
         ct_idx_247 * ct_idx_378) +
        -t1691 * t2249_tmp;
  ct_idx_295 = ((((ct_idx_253 * t1703 + t294 * t1770) - ct_idx_306 * t1772) +
                 ct_idx_247 * ct_idx_375) +
                t1691 * ct_idx_380) +
               -t1718 * t2248_tmp;
  ct_idx_720 = ((((t293 * t1704 + ct_idx_252 * t1773) + ct_idx_304 * t1771) -
                 t1719 * ct_idx_377) +
                ct_idx_248 * ct_idx_379) +
               -t1692 * t2247_tmp;
  t1799 = ((((ct_idx_254 * t1704 + t292 * t1771) - ct_idx_305 * t1773) +
            ct_idx_248 * ct_idx_376) +
           t1692 * ct_idx_378) +
          -t1719 * t2249_tmp;
  ct_idx_271 = ((((ct_idx_252 * t1705 + t293 * t1775) - ct_idx_304 * t1774) +
                 b_ct_idx_249 * ct_idx_377) +
                t1693 * ct_idx_379) +
               -t1723 * t2247_tmp;
  t137 = ((((t294 * t1705 + ct_idx_253 * t1774) + ct_idx_306 * t1775) -
           t1723 * ct_idx_375) +
          b_ct_idx_249 * ct_idx_380) +
         -t1693 * t2248_tmp;
  t289 = ((((t292 * t1706 + ct_idx_254 * t1778) + ct_idx_305 * t1776) -
           t1730 * ct_idx_376) +
          b_ct_idx_253 * ct_idx_378) +
         -t1697 * t2249_tmp;
  ct_idx_222 = ((((ct_idx_253 * t1706 + t294 * t1776) - ct_idx_306 * t1778) +
                 b_ct_idx_253 * ct_idx_375) +
                t1697 * ct_idx_380) +
               -t1730 * t2248_tmp;
  ct_idx_223 = ((((t293 * t1707 + ct_idx_252 * t1779) + ct_idx_304 * t1777) -
                 t1731 * ct_idx_377) +
                b_ct_idx_254 * ct_idx_379) +
               -t1698 * t2247_tmp;
  ct_idx_224 = ((((ct_idx_254 * t1707 + t292 * t1777) - ct_idx_305 * t1779) +
                 b_ct_idx_254 * ct_idx_376) +
                t1698 * ct_idx_378) +
               -t1731 * t2249_tmp;
  ct_idx_225 = ((((ct_idx_252 * t1708 + t293 * t1781) - ct_idx_304 * t1780) +
                 ct_idx_255 * ct_idx_377) +
                t1699 * ct_idx_379) +
               -t1735 * t2247_tmp;
  ct_idx_226 = ((((t294 * t1708 + ct_idx_253 * t1780) + ct_idx_306 * t1781) -
                 t1735 * ct_idx_375) +
                ct_idx_255 * ct_idx_380) +
               -t1699 * t2248_tmp;
  ct_idx_245_tmp = in24[0] * in20[2];
  b_ct_idx_245_tmp = in24[0] * in20[3];
  ct_idx_246_tmp = in20[1] * in24[2];
  b_ct_idx_246_tmp = in24[2] * in20[2];
  t78 = in21[1] * in18[1];
  t83 = in22[1] * in18[1];
  t275 = (((((ct_idx_435_tmp * t294 * 2.0 + t78 * ct_idx_252 * 2.0) * t1900 +
             -t1901 *
                 (ct_idx_435_tmp * ct_idx_253 * 2.0 - t78 * ct_idx_304 * 2.0)) -
            (t78 * t293 * 2.0 + ct_idx_435_tmp * ct_idx_306 * 2.0) * t1899) -
           (c_ct_idx_181_tmp * t294 * 2.0 + t83 * ct_idx_252 * 2.0) * t2248) +
          (t83 * t293 * 2.0 + c_ct_idx_181_tmp * ct_idx_306 * 2.0) * t2247) +
         t2249 * (c_ct_idx_181_tmp * ct_idx_253 * 2.0 - t83 * ct_idx_304 * 2.0);
  c_ct_idx_254 =
      ((((t1899 * (ct_idx_398_tmp * t293 * 2.0 -
                   b_ct_idx_398_tmp * ct_idx_254 * 2.0) +
          t1901 * (b_ct_idx_398_tmp * t292 * 2.0 -
                   ct_idx_398_tmp * ct_idx_304 * 2.0)) -
         (b_ct_idx_398_tmp * ct_idx_305 * 2.0 +
          ct_idx_398_tmp * ct_idx_252 * 2.0) *
             t1900) +
        -t2247 *
            (ct_idx_765_tmp * t293 * 2.0 - ct_idx_529_tmp * ct_idx_254 * 2.0)) +
       (ct_idx_765_tmp * ct_idx_252 * 2.0 + ct_idx_529_tmp * ct_idx_305 * 2.0) *
           t2248) +
      -t2249 *
          (ct_idx_529_tmp * t292 * 2.0 - ct_idx_765_tmp * ct_idx_304 * 2.0);
  b_ct_idx_255 =
      (((((t78 * t294 * 2.0 - ct_idx_435_tmp * ct_idx_252 * 2.0) * t1900 -
          (ct_idx_435_tmp * ct_idx_304 * 2.0 + t78 * ct_idx_253 * 2.0) *
              t1901) +
         t1899 * (ct_idx_435_tmp * t293 * 2.0 - t78 * ct_idx_306 * 2.0)) -
        (t83 * t294 * 2.0 - c_ct_idx_181_tmp * ct_idx_252 * 2.0) * t2248) +
       -t2247 * (c_ct_idx_181_tmp * t293 * 2.0 - t83 * ct_idx_306 * 2.0)) +
      (t83 * ct_idx_253 * 2.0 + c_ct_idx_181_tmp * ct_idx_304 * 2.0) * t2249;
  b_ct_idx_256 =
      ((((t1901 * (ct_idx_400_tmp * t292 * 2.0 -
                   b_ct_idx_400_tmp * ct_idx_253 * 2.0) +
          t1900 * (b_ct_idx_400_tmp * t294 * 2.0 -
                   ct_idx_400_tmp * ct_idx_305 * 2.0)) -
         (b_ct_idx_400_tmp * ct_idx_306 * 2.0 +
          ct_idx_400_tmp * ct_idx_254 * 2.0) *
             t1899) +
        -t2249 *
            (ct_idx_784_tmp * t292 * 2.0 - ct_idx_540_tmp * ct_idx_253 * 2.0)) +
       (ct_idx_784_tmp * ct_idx_254 * 2.0 + ct_idx_540_tmp * ct_idx_306 * 2.0) *
           t2247) +
      -t2248 *
          (ct_idx_540_tmp * t294 * 2.0 - ct_idx_784_tmp * ct_idx_305 * 2.0);
  ct_idx_257_tmp = in24[0] * in20[0];
  b_ct_idx_257_tmp = in24[0] * in20[1];
  ct_idx_258_tmp = in20[0] * in24[1];
  b_ct_idx_258_tmp = in24[1] * in20[2];
  ct_idx_259_tmp = in24[2] * in20[3];
  b_ct_idx_259_tmp = in20[0] * in24[2];
  ct_idx_272 =
      ((((((b_ct_idx_433_tmp * t292 * 2.0 +
            b_ct_idx_398_tmp * ct_idx_253 * 4.0) +
           ct_idx_433_tmp * ct_idx_304 * 2.0) *
              t1901 -
          ((b_ct_idx_398_tmp * t294 * 4.0 - ct_idx_433_tmp * ct_idx_252 * 2.0) +
           b_ct_idx_433_tmp * ct_idx_305 * 2.0) *
              t1900) -
         ((ct_idx_433_tmp * t293 * 2.0 + b_ct_idx_433_tmp * ct_idx_254 * 2.0) -
          b_ct_idx_398_tmp * ct_idx_306 * 4.0) *
             t1899) +
        ((b_ct_idx_180_tmp * t293 * 2.0 + ct_idx_180_tmp * ct_idx_254 * 2.0) -
         ct_idx_529_tmp * ct_idx_306 * 4.0) *
            t2247) +
       ((ct_idx_529_tmp * t294 * 4.0 - b_ct_idx_180_tmp * ct_idx_252 * 2.0) +
        ct_idx_180_tmp * ct_idx_305 * 2.0) *
           t2248) -
      ((ct_idx_180_tmp * t292 * 2.0 + ct_idx_529_tmp * ct_idx_253 * 4.0) +
       b_ct_idx_180_tmp * ct_idx_304 * 2.0) *
          t2249;
  ct_idx_273 =
      ((((((t78 * t292 * 4.0 + c_ct_idx_435_tmp * ct_idx_253 * 2.0) +
           b_ct_idx_435_tmp * ct_idx_304 * 2.0) *
              t1901 -
          ((c_ct_idx_435_tmp * t294 * 2.0 -
            b_ct_idx_435_tmp * ct_idx_252 * 2.0) +
           t78 * ct_idx_305 * 4.0) *
              t1900) -
         ((b_ct_idx_435_tmp * t293 * 2.0 + t78 * ct_idx_254 * 4.0) -
          c_ct_idx_435_tmp * ct_idx_306 * 2.0) *
             t1899) +
        ((ct_idx_181_tmp * t293 * 2.0 + t83 * ct_idx_254 * 4.0) -
         b_ct_idx_181_tmp * ct_idx_306 * 2.0) *
            t2247) +
       ((b_ct_idx_181_tmp * t294 * 2.0 - ct_idx_181_tmp * ct_idx_252 * 2.0) +
        t83 * ct_idx_305 * 4.0) *
           t2248) -
      ((t83 * t292 * 4.0 + b_ct_idx_181_tmp * ct_idx_253 * 2.0) +
       ct_idx_181_tmp * ct_idx_304 * 2.0) *
          t2249;
  t277 = in18[0] * t2562 * 2.0;
  ct_idx_118 = in18[1] * t2562 * 2.0;
  ct_idx_169 = in18[2] * t2562;
  ct_idx_282 = ct_idx_169 * 2.0;
  t3 = in18[3] * t2562;
  ct_idx_281 = t3 * 2.0;
  t4 = ((((t1704 * t1899 - t1771 * t1901) - t1773 * t1900) - t1692 * t2247) +
        ct_idx_248 * t2248) +
       t1719 * t2249;
  t2 = in18[1] * t4;
  ct_idx_283 = t2 * 2.0;
  t6 = in18[2] * t4 * 2.0;
  t7 = ((((t1705 * t1900 - t1774 * t1901) - t1775 * t1899) - t1693 * t2248) +
        t1723 * t2247) +
       b_ct_idx_249 * t2249;
  t57 = in18[2] * t7;
  t59 = t57 * 2.0;
  t62 = in18[3] * t7 * 2.0;
  t141 = ct_idx_93_tmp * t2562;
  t276 = ct_idx_98_tmp * t4 * 2.0;
  t278 = ct_idx_95_tmp * t7 * 2.0;
  t79 = t297 * t2562;
  ct_idx_296 = ct_idx_257 * t2562;
  t82 = t525 * t2562;
  t279 = in23[1] * in20[3];
  ct_idx_119 = in23[1] * in20[1];
  ct_idx_166 = in23[0] * in20[0];
  ct_idx_168 = in23[0] * in20[1];
  ct_idx_302 = in20[0] * in23[1];
  ct_idx_301 = in23[1] * in20[2];
  ct_idx_300 = in23[2] * in20[3];
  ct_idx_251 = in20[0] * in23[2];
  ct_idx_250 =
      ((in23[3] * t87 -
        (ct_idx_431_tmp * t293 * 2.0 + ct_idx_440_tmp * ct_idx_254 * 2.0) *
            t2421) +
       (ct_idx_440_tmp * t292 * 2.0 + ct_idx_431_tmp * ct_idx_304 * 2.0) *
           t2419) +
      t2420 * (ct_idx_431_tmp * ct_idx_252 * 2.0 -
               ct_idx_440_tmp * ct_idx_305 * 2.0);
  ct_idx_249 =
      ((in23[5] * t85 -
        (ct_idx_444_tmp * t292 * 2.0 + ct_idx_449_tmp * ct_idx_253 * 2.0) *
            t2419) +
       (ct_idx_449_tmp * t294 * 2.0 + ct_idx_444_tmp * ct_idx_305 * 2.0) *
           t2420) +
      t2421 * (ct_idx_444_tmp * ct_idx_254 * 2.0 -
               ct_idx_449_tmp * ct_idx_306 * 2.0);
  t78 = in20[0] * in24[3];
  t83 = in20[1] * in24[3];
  t290 = ((in23[3] * t85 +
           ((ct_idx_431_tmp * t294 * 4.0 - t78 * ct_idx_252 * 2.0) +
            t83 * ct_idx_305 * 2.0) *
               t2420) +
          ((t78 * t293 * 2.0 + t83 * ct_idx_254 * 2.0) -
           ct_idx_431_tmp * ct_idx_306 * 4.0) *
              t2421) -
         ((t83 * t292 * 2.0 + ct_idx_431_tmp * ct_idx_253 * 4.0) +
          t78 * ct_idx_304 * 2.0) *
             t2419;
  t139 = ((in23[3] * t2228 +
           ((t78 * t292 * 2.0 + ct_idx_440_tmp * ct_idx_253 * 4.0) -
            t83 * ct_idx_304 * 2.0) *
               t2419) +
          ((t83 * t293 * 2.0 - t78 * ct_idx_254 * 2.0) +
           ct_idx_440_tmp * ct_idx_306 * 4.0) *
              t2421) -
         ((ct_idx_440_tmp * t294 * 4.0 + t83 * ct_idx_252 * 2.0) +
          t78 * ct_idx_305 * 2.0) *
             t2420;
  t78 = in20[0] * in24[4];
  t83 = in20[2] * in24[4];
  t5 = ((in23[4] * t85 + ((t83 * t294 * 2.0 - t78 * ct_idx_252 * 2.0) +
                          ct_idx_39_tmp * ct_idx_305 * 4.0) *
                             t2420) +
        ((t78 * t293 * 2.0 + ct_idx_39_tmp * ct_idx_254 * 4.0) -
         t83 * ct_idx_306 * 2.0) *
            t2421) -
       ((ct_idx_39_tmp * t292 * 4.0 + t83 * ct_idx_253 * 2.0) +
        t78 * ct_idx_304 * 2.0) *
           t2419;
  t274 = ((in23[4] * t87 +
           ((ct_idx_447_tmp * t292 * 4.0 - t78 * ct_idx_253 * 2.0) +
            t83 * ct_idx_304 * 2.0) *
               t2419) +
          ((t78 * t294 * 2.0 + t83 * ct_idx_252 * 2.0) -
           ct_idx_447_tmp * ct_idx_305 * 4.0) *
              t2420) -
         ((t83 * t293 * 2.0 + ct_idx_447_tmp * ct_idx_254 * 4.0) +
          t78 * ct_idx_306 * 2.0) *
             t2421;
  t78 = in20[0] * in24[5];
  t83 = in20[3] * in24[5];
  ct_idx_117 =
      ((in23[5] * t2228 + ((t78 * t292 * 2.0 + t83 * ct_idx_253 * 2.0) -
                           ct_idx_444_tmp * ct_idx_304 * 4.0) *
                              t2419) +
       ((ct_idx_444_tmp * t293 * 4.0 - t78 * ct_idx_254 * 2.0) +
        t83 * ct_idx_306 * 2.0) *
           t2421) -
      ((t83 * t294 * 2.0 + ct_idx_444_tmp * ct_idx_252 * 4.0) +
       t78 * ct_idx_305 * 2.0) *
          t2420;
  t1710 = ((in23[5] * t87 + ((t83 * t292 * 2.0 - t78 * ct_idx_253 * 2.0) +
                             ct_idx_449_tmp * ct_idx_304 * 4.0) *
                                t2419) +
           ((t78 * t294 * 2.0 + ct_idx_449_tmp * ct_idx_252 * 4.0) -
            t83 * ct_idx_305 * 2.0) *
               t2420) -
          ((ct_idx_449_tmp * t293 * 4.0 + t83 * ct_idx_254 * 2.0) +
           t78 * ct_idx_306 * 2.0) *
              t2421;
  t1711 = in20[0] * t80 * 2.0;
  ct_idx_294 = in20[1] * t80 * 2.0;
  ct_idx_722 = in20[2] * t80;
  ct_idx_721 = ct_idx_722 * 2.0;
  ct_idx_960 = in20[3] * t80;
  t1709 = ct_idx_960 * 2.0;
  ct_idx_463 = ((in23[4] * (((in8[0] * t147 + in8[1] * t148) + in8[2] * t98) -
                            in8[3] * t146) +
                 ((in24[4] * t293 * t298 + in24[4] * ct_idx_254 * t527) +
                  in24[4] * ct_idx_261 * ct_idx_306) *
                     t2421) -
                ((in24[4] * t292 * t527 + t84 * ct_idx_304) -
                 in24[4] * ct_idx_253 * ct_idx_261) *
                    t2419) -
               ((t84 * ct_idx_252 + in24[4] * t294 * ct_idx_261) -
                in24[4] * ct_idx_305 * t527) *
                   t2420;
  t1557 = in20[1] * ct_idx_463;
  t1558 = t1557 * -2.0;
  ct_idx_462 = in20[2] * ct_idx_463 * -2.0;
  t90 = ((in23[5] * (((in8[0] * t148 + in8[2] * t146) + in8[3] * t98) -
                     in8[1] * t147) +
          ((in24[5] * t294 * t299 + in24[5] * ct_idx_252 * t526) +
           in24[5] * ct_idx_259 * ct_idx_305) *
              t2420) -
         ((in24[5] * t292 * ct_idx_259 + t89 * ct_idx_253) -
          in24[5] * ct_idx_304 * t526) *
             t2419) -
        ((in24[5] * t293 * t526 + t89 * ct_idx_306) -
         in24[5] * ct_idx_254 * ct_idx_259) *
            t2421;
  t95 = in20[2] * t90;
  t91 = t95 * -2.0;
  t94 = in20[3] * t90 * -2.0;
  t92 = (((((ct_idx_419_tmp * t294 * 2.0 + ct_idx_79_tmp * ct_idx_252 * 2.0) *
                t2484 +
            -t2483 * (ct_idx_419_tmp * ct_idx_253 * 2.0 -
                      ct_idx_79_tmp * ct_idx_304 * 2.0)) -
           (ct_idx_79_tmp * t293 * 2.0 + ct_idx_419_tmp * ct_idx_306 * 2.0) *
               t2485) -
          (t279 * t294 * 2.0 + ct_idx_119 * ct_idx_252 * 2.0) * t2599) +
         (ct_idx_119 * t293 * 2.0 + t279 * ct_idx_306 * 2.0) * t2598) +
        t2600 * (t279 * ct_idx_253 * 2.0 - ct_idx_119 * ct_idx_304 * 2.0);
  t93 = ((((-t2485 * (b_ct_idx_245_tmp * t293 * 2.0 -
                      ct_idx_245_tmp * ct_idx_254 * 2.0) +
            -t2483 * (ct_idx_245_tmp * t292 * 2.0 -
                      b_ct_idx_245_tmp * ct_idx_304 * 2.0)) +
           (ct_idx_245_tmp * ct_idx_305 * 2.0 +
            b_ct_idx_245_tmp * ct_idx_252 * 2.0) *
               t2484) +
          t2598 * (ct_idx_55_tmp * t293 * 2.0 -
                   ct_idx_679_tmp * ct_idx_254 * 2.0)) +
         t2600 *
             (ct_idx_679_tmp * t292 * 2.0 - ct_idx_55_tmp * ct_idx_304 * 2.0)) -
        (ct_idx_679_tmp * ct_idx_305 * 2.0 + ct_idx_55_tmp * ct_idx_252 * 2.0) *
            t2599;
  t96 = ((((-t2483 * (b_ct_idx_246_tmp * t292 * 2.0 -
                      ct_idx_246_tmp * ct_idx_253 * 2.0) +
            -t2484 * (ct_idx_246_tmp * t294 * 2.0 -
                      b_ct_idx_246_tmp * ct_idx_305 * 2.0)) +
           (ct_idx_246_tmp * ct_idx_306 * 2.0 +
            b_ct_idx_246_tmp * ct_idx_254 * 2.0) *
               t2485) +
          t2600 * (ct_idx_59_tmp * t292 * 2.0 -
                   ct_idx_690_tmp * ct_idx_253 * 2.0)) +
         t2599 *
             (ct_idx_690_tmp * t294 * 2.0 - ct_idx_59_tmp * ct_idx_305 * 2.0)) -
        (ct_idx_690_tmp * ct_idx_306 * 2.0 + ct_idx_59_tmp * ct_idx_254 * 2.0) *
            t2598;
  t101 =
      ((((((b_ct_idx_257_tmp * t292 * 2.0 + ct_idx_245_tmp * ct_idx_253 * 4.0) +
           ct_idx_257_tmp * ct_idx_304 * 2.0) *
              t2483 -
          ((ct_idx_245_tmp * t294 * 4.0 - ct_idx_257_tmp * ct_idx_252 * 2.0) +
           b_ct_idx_257_tmp * ct_idx_305 * 2.0) *
              t2484) -
         ((ct_idx_257_tmp * t293 * 2.0 + b_ct_idx_257_tmp * ct_idx_254 * 2.0) -
          ct_idx_245_tmp * ct_idx_306 * 4.0) *
             t2485) +
        ((ct_idx_679_tmp * t294 * 4.0 - ct_idx_166 * ct_idx_252 * 2.0) +
         ct_idx_168 * ct_idx_305 * 2.0) *
            t2599) +
       ((ct_idx_166 * t293 * 2.0 + ct_idx_168 * ct_idx_254 * 2.0) -
        ct_idx_679_tmp * ct_idx_306 * 4.0) *
           t2598) -
      ((ct_idx_168 * t292 * 2.0 + ct_idx_679_tmp * ct_idx_253 * 4.0) +
       ct_idx_166 * ct_idx_304 * 2.0) *
          t2600;
  t97 =
      ((((((ct_idx_79_tmp * t292 * 4.0 + b_ct_idx_258_tmp * ct_idx_253 * 2.0) +
           ct_idx_258_tmp * ct_idx_304 * 2.0) *
              t2483 -
          ((b_ct_idx_258_tmp * t294 * 2.0 - ct_idx_258_tmp * ct_idx_252 * 2.0) +
           ct_idx_79_tmp * ct_idx_305 * 4.0) *
              t2484) -
         ((ct_idx_258_tmp * t293 * 2.0 + ct_idx_79_tmp * ct_idx_254 * 4.0) -
          b_ct_idx_258_tmp * ct_idx_306 * 2.0) *
             t2485) +
        ((ct_idx_301 * t294 * 2.0 - ct_idx_302 * ct_idx_252 * 2.0) +
         ct_idx_119 * ct_idx_305 * 4.0) *
            t2599) +
       ((ct_idx_302 * t293 * 2.0 + ct_idx_119 * ct_idx_254 * 4.0) -
        ct_idx_301 * ct_idx_306 * 2.0) *
           t2598) -
      ((ct_idx_119 * t292 * 4.0 + ct_idx_301 * ct_idx_253 * 2.0) +
       ct_idx_302 * ct_idx_304 * 2.0) *
          t2600;
  t100 = in20[0] * t2713 * 2.0;
  t98 = in20[1] * t2713 * 2.0;
  t99 = in20[2] * t2713;
  t144 = t99 * 2.0;
  t145 = in20[3] * t2713;
  t146 = t145 * 2.0;
  t147 = in20[2] * t2721_tmp * -2.0;
  t148 = in20[0] * t2715 * 2.0;
  t149 = in20[1] * t2715;
  ct_idx_138 = t149 * 2.0;
  ct_idx_160 = in20[2] * t2715;
  ct_idx_188 = ct_idx_160 * 2.0;
  ct_idx_163 = in20[3] * t2715 * 2.0;
  ct_idx_189 = in19[1] * t2723;
  ct_idx_164 = -ct_idx_111_tmp * t2713;
  ct_idx_165 = -ct_idx_130_tmp * t2715;
  ct_idx_190 = t300 * t2713;
  t138 = t299 * t2715;
  t135 = t2713 * ct_idx_260;
  t136 = t2715 * ct_idx_259;
  t1785 = t2713 * t528;
  t1556 = t2715 * t526;
  t85 = ((((-(t294 * t1773) + ct_idx_253 * t1771) - ct_idx_306 * t1704) +
          t1692 * ct_idx_375) +
         t1719 * ct_idx_380) +
        ct_idx_248 * t2248_tmp;
  b_jacobian_WRh_to_oTg1_oTg2[0] =
      (-t296 * t137 - ct_idx_295 * t525) + ct_idx_258 * t85;
  b_jacobian_WRh_to_oTg1_oTg2[1] =
      (ct_idx_295 * t297 + t137 * ct_idx_256) + t524 * t85;
  b_jacobian_WRh_to_oTg1_oTg2[2] =
      (-ct_idx_257 * ct_idx_295 + t137 * t523) - t295 * t85;
  t88 = in17[2] * ct_idx_256 - in17[1] * t523;
  b_jacobian_WRh_to_oTg1_oTg2[3] =
      (-b_ct_idx_90 * ct_idx_295 - t137 * t88) - b_ct_idx_96 * t85;
  t86 = in17[0] * ct_idx_257 - in17[2] * t525;
  b_jacobian_WRh_to_oTg1_oTg2[4] =
      ((((-t295 * t4 + ct_idx_296) + t523 * t7) - b_ct_idx_94 * t137) +
       ct_idx_295 * t86) +
      b_ct_idx_89 * t85;
  t87 = in17[1] * ct_idx_258 - in17[0] * t524;
  b_jacobian_WRh_to_oTg1_oTg2[5] =
      ((((t79 + -ct_idx_256 * t7) + -t524 * t4) + b_ct_idx_88 * t137) +
       b_ct_idx_95 * ct_idx_295) -
      t87 * t85;
  t85 = ((((-(t292 * t1774) + ct_idx_254 * t1775) - ct_idx_305 * t1705) +
          t1693 * ct_idx_376) +
         t1723 * ct_idx_378) +
        b_ct_idx_249 * t2249_tmp;
  b_jacobian_WRh_to_oTg1_oTg2[6] =
      (-ct_idx_258 * t1799 + t81 * t525) - t296 * t85;
  b_jacobian_WRh_to_oTg1_oTg2[7] =
      (-t297 * t81 - t1799 * t524) + ct_idx_256 * t85;
  b_jacobian_WRh_to_oTg1_oTg2[8] =
      (t1799 * t295 + t81 * ct_idx_257) + t523 * t85;
  b_jacobian_WRh_to_oTg1_oTg2[9] =
      ((((-ct_idx_296 + -t523 * t7) + t295 * t4) + b_ct_idx_90 * t81) +
       b_ct_idx_96 * t1799) -
      t88 * t85;
  b_jacobian_WRh_to_oTg1_oTg2[10] =
      (-b_ct_idx_89 * t1799 - t81 * t86) - b_ct_idx_94 * t85;
  b_jacobian_WRh_to_oTg1_oTg2[11] =
      ((((-t296 * t7 + t82) + ct_idx_258 * t4) - b_ct_idx_95 * t81) +
       t1799 * t87) +
      b_ct_idx_88 * t85;
  t85 = ((((-(t293 * t1772) + ct_idx_252 * t1770) - ct_idx_304 * t1703) +
          t1691 * ct_idx_377) +
         t1718 * ct_idx_379) +
        ct_idx_247 * t2247_tmp;
  b_jacobian_WRh_to_oTg1_oTg2[12] =
      (ct_idx_271 * t296 + ct_idx_720 * ct_idx_258) + t525 * t85;
  b_jacobian_WRh_to_oTg1_oTg2[13] =
      (-ct_idx_256 * ct_idx_271 + ct_idx_720 * t524) - t297 * t85;
  b_jacobian_WRh_to_oTg1_oTg2[14] =
      (-t295 * ct_idx_720 - ct_idx_271 * t523) + ct_idx_257 * t85;
  b_jacobian_WRh_to_oTg1_oTg2[15] =
      ((((-t79 + ct_idx_256 * t7) + t524 * t4) - b_ct_idx_96 * ct_idx_720) +
       ct_idx_271 * t88) +
      b_ct_idx_90 * t85;
  b_jacobian_WRh_to_oTg1_oTg2[16] =
      ((((-ct_idx_258 * t4 - t82) + t296 * t7) + b_ct_idx_89 * ct_idx_720) +
       b_ct_idx_94 * ct_idx_271) -
      t86 * t85;
  b_jacobian_WRh_to_oTg1_oTg2[17] =
      (-b_ct_idx_88 * ct_idx_271 - ct_idx_720 * t87) - b_ct_idx_95 * t85;
  t85 =
      (((((b_ct_idx_398_tmp * t293 * 2.0 + ct_idx_398_tmp * ct_idx_254 * 2.0) *
              t1899 -
          (ct_idx_398_tmp * t292 * 2.0 + b_ct_idx_398_tmp * ct_idx_304 * 2.0) *
              t1901) +
         -t1900 * (b_ct_idx_398_tmp * ct_idx_252 * 2.0 -
                   ct_idx_398_tmp * ct_idx_305 * 2.0)) -
        (ct_idx_529_tmp * t293 * 2.0 + ct_idx_765_tmp * ct_idx_254 * 2.0) *
            t2247) +
       (ct_idx_765_tmp * t292 * 2.0 + ct_idx_529_tmp * ct_idx_304 * 2.0) *
           t2249) +
      t2248 * (ct_idx_529_tmp * ct_idx_252 * 2.0 -
               ct_idx_765_tmp * ct_idx_305 * 2.0);
  t81 =
      (((((b_ct_idx_400_tmp * t292 * 2.0 + ct_idx_400_tmp * ct_idx_253 * 2.0) *
              t1901 -
          (ct_idx_400_tmp * t294 * 2.0 + b_ct_idx_400_tmp * ct_idx_305 * 2.0) *
              t1900) +
         -t1899 * (b_ct_idx_400_tmp * ct_idx_254 * 2.0 -
                   ct_idx_400_tmp * ct_idx_306 * 2.0)) -
        (ct_idx_540_tmp * t292 * 2.0 + ct_idx_784_tmp * ct_idx_253 * 2.0) *
            t2249) +
       (ct_idx_784_tmp * t294 * 2.0 + ct_idx_540_tmp * ct_idx_305 * 2.0) *
           t2248) +
      t2247 * (ct_idx_540_tmp * ct_idx_254 * 2.0 -
               ct_idx_784_tmp * ct_idx_306 * 2.0);
  t84 = in18[3] * t4;
  b_jacobian_WRh_to_oTg1_oTg2[18] =
      (((t84 * -2.0 + t59) + t275 * ct_idx_258) - t296 * t81) + t525 * t85;
  t89 = in18[1] * t7;
  b_jacobian_WRh_to_oTg1_oTg2[19] =
      (((ct_idx_281 + t89 * -2.0) + t275 * t524) - t297 * t85) +
      ct_idx_256 * t81;
  b_jacobian_WRh_to_oTg1_oTg2[20] =
      (((-ct_idx_282 + ct_idx_283) - t275 * t295) + ct_idx_257 * t85) +
      t523 * t81;
  t82 = ((in6[3] * ct_idx_394 - in6[0] * ct_idx_103) + in6[1] * ct_idx_101) -
        in6[2] * ct_idx_100;
  t80 = ((-(b_ct_idx_119_tmp * t292 * 2.0 + ct_idx_119_tmp * ct_idx_253 * 2.0) *
              ct_idx_381_tmp +
          (b_ct_idx_119_tmp * ct_idx_254 * 2.0 -
           ct_idx_119_tmp * ct_idx_306 * 2.0) *
              ct_idx_391_tmp) +
         in21[5] * t82) +
        (ct_idx_119_tmp * t294 * 2.0 + b_ct_idx_119_tmp * ct_idx_305 * 2.0) *
            ct_idx_390_tmp;
  b_jacobian_WRh_to_oTg1_oTg2[21] = (((((((((ct_idx_206 + ct_idx_213) + t278) -
                                           t2562 * (ct_idx_96 + ct_idx_102)) +
                                          in17[1] * ct_idx_283) -
                                         ct_idx_513 * ct_idx_258) -
                                        ct_idx_174 * t525) -
                                       b_ct_idx_96 * t275) +
                                      t296 * t80) +
                                     b_ct_idx_90 * t85) -
                                    t88 * t81;
  b_jacobian_WRh_to_oTg1_oTg2[22] =
      (((((((((ct_idx_208 + ct_idx_210) + t141) + in17[2] * t59) +
            ct_idx_174 * t297) -
           ct_idx_513 * t524) +
          b_ct_idx_89 * t275) -
         ct_idx_256 * t80) -
        b_ct_idx_94 * t81) -
       (ct_idx_90 + ct_idx_102) * t4) -
      t86 * t85;
  b_jacobian_WRh_to_oTg1_oTg2[23] =
      (((((((((ct_idx_204 + ct_idx_211) + -ct_idx_149 * t2562) + t276) +
            ct_idx_513 * t295) -
           ct_idx_174 * ct_idx_257) -
          t523 * t80) -
         t275 * t87) +
        b_ct_idx_88 * t81) -
       b_ct_idx_95 * t85) -
      (ct_idx_90 + ct_idx_96) * t7;
  t85 =
      ((((((ct_idx_436_tmp * t294 * 2.0 + b_ct_idx_400_tmp * ct_idx_252 * 4.0) +
           b_ct_idx_436_tmp * ct_idx_305 * 2.0) *
              t1900 -
          ((b_ct_idx_436_tmp * t292 * 2.0 + ct_idx_436_tmp * ct_idx_253 * 2.0) -
           b_ct_idx_400_tmp * ct_idx_304 * 4.0) *
              t1901) -
         ((b_ct_idx_400_tmp * t293 * 4.0 -
           b_ct_idx_436_tmp * ct_idx_254 * 2.0) +
          ct_idx_436_tmp * ct_idx_306 * 2.0) *
             t1899) -
        ((ct_idx_182_tmp * t294 * 2.0 + ct_idx_540_tmp * ct_idx_252 * 4.0) +
         b_ct_idx_182_tmp * ct_idx_305 * 2.0) *
            t2248) +
       ((ct_idx_540_tmp * t293 * 4.0 - b_ct_idx_182_tmp * ct_idx_254 * 2.0) +
        ct_idx_182_tmp * ct_idx_306 * 2.0) *
           t2247) +
      ((b_ct_idx_182_tmp * t292 * 2.0 + ct_idx_182_tmp * ct_idx_253 * 2.0) -
       ct_idx_540_tmp * ct_idx_304 * 4.0) *
          t2249;
  b_jacobian_WRh_to_oTg1_oTg2[24] =
      (((t6 + t62) + t296 * t85) + ct_idx_273 * ct_idx_258) -
      c_ct_idx_254 * t525;
  t81 = in18[0] * t7;
  b_jacobian_WRh_to_oTg1_oTg2[25] =
      ((((ct_idx_282 + t81 * -2.0) - t2 * 4.0) - ct_idx_256 * t85) +
       c_ct_idx_254 * t297) +
      ct_idx_273 * t524;
  t80 = in18[0] * t4;
  b_jacobian_WRh_to_oTg1_oTg2[26] =
      ((((ct_idx_281 + t80 * 2.0) - t89 * 4.0) - t523 * t85) -
       c_ct_idx_254 * ct_idx_257) -
      ct_idx_273 * t295;
  t79 = ((in21[3] * t1717 - (b_ct_idx_132_tmp * t293 * 2.0 -
                             c_ct_idx_132_tmp * ct_idx_254 * 2.0) *
                                ct_idx_391_tmp) -
         (c_ct_idx_132_tmp * t292 * 2.0 - b_ct_idx_132_tmp * ct_idx_304 * 2.0) *
             ct_idx_381_tmp) +
        (b_ct_idx_132_tmp * ct_idx_252 * 2.0 +
         c_ct_idx_132_tmp * ct_idx_305 * 2.0) *
            ct_idx_390_tmp;
  t83 = ((-((b_ct_idx_189_tmp * t294 * 2.0 +
             b_ct_idx_119_tmp * ct_idx_252 * 4.0) +
            ct_idx_189_tmp * ct_idx_305 * 2.0) *
              ct_idx_390_tmp +
          in21[5] * ct_idx_513_tmp) +
         ((ct_idx_189_tmp * t292 * 2.0 + b_ct_idx_189_tmp * ct_idx_253 * 2.0) -
          b_ct_idx_119_tmp * ct_idx_304 * 4.0) *
             ct_idx_381_tmp) +
        ((b_ct_idx_119_tmp * t293 * 4.0 - ct_idx_189_tmp * ct_idx_254 * 2.0) +
         b_ct_idx_189_tmp * ct_idx_306 * 2.0) *
            ct_idx_391_tmp;
  t78 = ((-((b_ct_idx_133_tmp * t292 * 4.0 +
             b_ct_idx_187_tmp * ct_idx_253 * 2.0) +
            ct_idx_187_tmp * ct_idx_304 * 2.0) *
              ct_idx_381_tmp +
          in21[4] * t82) +
         ((b_ct_idx_187_tmp * t294 * 2.0 - ct_idx_187_tmp * ct_idx_252 * 2.0) +
          b_ct_idx_133_tmp * ct_idx_305 * 4.0) *
             ct_idx_390_tmp) +
        ((ct_idx_187_tmp * t293 * 2.0 + b_ct_idx_133_tmp * ct_idx_254 * 4.0) -
         b_ct_idx_187_tmp * ct_idx_306 * 2.0) *
            ct_idx_391_tmp;
  b_jacobian_WRh_to_oTg1_oTg2[27] =
      (((((((((ct_idx_212 + t291) + t525 * t79) + t88 * t85) -
            b_ct_idx_90 * c_ct_idx_254) -
           b_ct_idx_96 * ct_idx_273) -
          t2562 * (ct_idx_97 - ct_idx_98)) +
         (ct_idx_89 + ct_idx_95_tmp * 4.0) * t4) +
        (ct_idx_91 - ct_idx_92_tmp * 4.0) * t7) -
       t296 * t83) +
      -ct_idx_258 * t78;
  b_jacobian_WRh_to_oTg1_oTg2[28] =
      ((((((((((-ct_idx_204 + ct_idx_207) + b_ct_idx_94 * t85) - t297 * t79) -
             (ct_idx_88 - ct_idx_97) * t4) +
            ct_idx_211_tmp * 4.0) +
           b_ct_idx_89 * ct_idx_273) +
          c_ct_idx_254 * t86) +
         ct_idx_256 * t83) -
        t524 * t78) +
       (ct_idx_94 + ct_idx_102) * t7) +
      ct_idx_149_tmp * t2562 * -2.0;
  b_jacobian_WRh_to_oTg1_oTg2[29] =
      ((((((((((-ct_idx_205 + ct_idx_210) + t141) - b_ct_idx_88 * t85) +
             ct_idx_257 * t79) +
            ct_idx_208_tmp * 4.0) +
           b_ct_idx_95 * c_ct_idx_254) -
          ct_idx_273 * t87) +
         t295 * t78) +
        t523 * t83) -
       (ct_idx_94 + ct_idx_96) * t4) -
      (ct_idx_88 + ct_idx_98) * t7;
  t85 =
      ((((((ct_idx_400_tmp * t293 * 4.0 + ct_idx_436_tmp * ct_idx_254 * 2.0) +
           b_ct_idx_436_tmp * ct_idx_306 * 2.0) *
              t1899 -
          ((ct_idx_436_tmp * t292 * 2.0 - b_ct_idx_436_tmp * ct_idx_253 * 2.0) +
           ct_idx_400_tmp * ct_idx_304 * 4.0) *
              t1901) -
         ((b_ct_idx_436_tmp * t294 * 2.0 + ct_idx_400_tmp * ct_idx_252 * 4.0) -
          ct_idx_436_tmp * ct_idx_305 * 2.0) *
             t1900) -
        ((ct_idx_784_tmp * t293 * 4.0 + ct_idx_182_tmp * ct_idx_254 * 2.0) +
         b_ct_idx_182_tmp * ct_idx_306 * 2.0) *
            t2247) +
       ((b_ct_idx_182_tmp * t294 * 2.0 + ct_idx_784_tmp * ct_idx_252 * 4.0) -
        ct_idx_182_tmp * ct_idx_305 * 2.0) *
           t2248) +
      ((ct_idx_182_tmp * t292 * 2.0 - b_ct_idx_182_tmp * ct_idx_253 * 2.0) +
       ct_idx_784_tmp * ct_idx_304 * 4.0) *
          t2249;
  b_jacobian_WRh_to_oTg1_oTg2[30] =
      ((((ct_idx_283 + t81 * 2.0) - t296 * t85) - ct_idx_169 * 4.0) -
       b_ct_idx_255 * ct_idx_258) -
      ct_idx_272 * t525;
  b_jacobian_WRh_to_oTg1_oTg2[31] =
      (((ct_idx_118 + t62) + ct_idx_256 * t85) + ct_idx_272 * t297) -
      b_ct_idx_255 * t524;
  b_jacobian_WRh_to_oTg1_oTg2[32] =
      ((((-t277 + t84 * 2.0) - t57 * 4.0) + t523 * t85) + b_ct_idx_255 * t295) -
      ct_idx_272 * ct_idx_257;
  t81 = ((in21[4] * t1717 + -(b_ct_idx_133_tmp * t294 * 2.0 -
                              c_ct_idx_133_tmp * ct_idx_252 * 2.0) *
                                ct_idx_390_tmp) -
         (c_ct_idx_133_tmp * t293 * 2.0 - b_ct_idx_133_tmp * ct_idx_306 * 2.0) *
             ct_idx_391_tmp) +
        (b_ct_idx_133_tmp * ct_idx_253 * 2.0 +
         c_ct_idx_133_tmp * ct_idx_304 * 2.0) *
            ct_idx_381_tmp;
  t79 = ((-((b_ct_idx_183_tmp * t292 * 2.0 +
             c_ct_idx_132_tmp * ct_idx_253 * 4.0) +
            ct_idx_183_tmp * ct_idx_304 * 2.0) *
              ct_idx_381_tmp +
          in21[3] * t82) +
         ((c_ct_idx_132_tmp * t294 * 4.0 - ct_idx_183_tmp * ct_idx_252 * 2.0) +
          b_ct_idx_183_tmp * ct_idx_305 * 2.0) *
             ct_idx_390_tmp) +
        ((ct_idx_183_tmp * t293 * 2.0 + b_ct_idx_183_tmp * ct_idx_254 * 2.0) -
         c_ct_idx_132_tmp * ct_idx_306 * 4.0) *
            ct_idx_391_tmp;
  b_jacobian_WRh_to_oTg1_oTg2[33] =
      ((((((((((-ct_idx_207 + ct_idx_211) + t276) -
              t2562 * (ct_idx_89 + ct_idx_95)) -
             t88 * t85) +
            ct_idx_204_tmp * 4.0) +
           ct_idx_193 * t296) -
          b_ct_idx_90 * ct_idx_272) +
         b_ct_idx_96 * b_ct_idx_255) +
        ct_idx_258 * t81) +
       t525 * t79) -
      (ct_idx_99 + ct_idx_102) * t7;
  b_jacobian_WRh_to_oTg1_oTg2[34] =
      (((((((((ct_idx_209 + t291) - b_ct_idx_94 * t85) -
             ct_idx_193 * ct_idx_256) -
            b_ct_idx_89 * b_ct_idx_255) +
           ct_idx_272 * t86) +
          (ct_idx_91 + ct_idx_93_tmp_tmp * 4.0) * t7) +
         t524 * t81) -
        t297 * t79) +
       (ct_idx_95 + ct_idx_149) * t4) +
      t2562 * (ct_idx_88 - ct_idx_97_tmp * 4.0);
  b_jacobian_WRh_to_oTg1_oTg2[35] =
      ((((((((((ct_idx_203 - ct_idx_206) + -ct_idx_92 * t4) +
              b_ct_idx_88 * t85) +
             t2562 * (ct_idx_90 + ct_idx_99)) +
            ct_idx_213_tmp * 4.0) -
           ct_idx_193 * t523) +
          b_ct_idx_95 * ct_idx_272) +
         b_ct_idx_255 * t87) -
        t295 * t81) +
       ct_idx_257 * t79) -
      (ct_idx_89 + ct_idx_149) * t7;
  t85 =
      ((((((ct_idx_398_tmp * t294 * 4.0 + b_ct_idx_433_tmp * ct_idx_252 * 2.0) +
           ct_idx_433_tmp * ct_idx_305 * 2.0) *
              t1900 -
          ((ct_idx_433_tmp * t292 * 2.0 + ct_idx_398_tmp * ct_idx_253 * 4.0) -
           b_ct_idx_433_tmp * ct_idx_304 * 2.0) *
              t1901) -
         ((b_ct_idx_433_tmp * t293 * 2.0 - ct_idx_433_tmp * ct_idx_254 * 2.0) +
          ct_idx_398_tmp * ct_idx_306 * 4.0) *
             t1899) -
        ((ct_idx_765_tmp * t294 * 4.0 + ct_idx_180_tmp * ct_idx_252 * 2.0) +
         b_ct_idx_180_tmp * ct_idx_305 * 2.0) *
            t2248) +
       ((ct_idx_180_tmp * t293 * 2.0 - b_ct_idx_180_tmp * ct_idx_254 * 2.0) +
        ct_idx_765_tmp * ct_idx_306 * 4.0) *
           t2247) +
      ((b_ct_idx_180_tmp * t292 * 2.0 + ct_idx_765_tmp * ct_idx_253 * 4.0) -
       ct_idx_180_tmp * ct_idx_304 * 2.0) *
          t2249;
  t81 =
      ((((((c_ct_idx_435_tmp * t293 * 2.0 + ct_idx_435_tmp * ct_idx_254 * 4.0) +
           b_ct_idx_435_tmp * ct_idx_306 * 2.0) *
              t1899 -
          ((ct_idx_435_tmp * t292 * 4.0 - b_ct_idx_435_tmp * ct_idx_253 * 2.0) +
           c_ct_idx_435_tmp * ct_idx_304 * 2.0) *
              t1901) -
         ((b_ct_idx_435_tmp * t294 * 2.0 +
           c_ct_idx_435_tmp * ct_idx_252 * 2.0) -
          ct_idx_435_tmp * ct_idx_305 * 4.0) *
             t1900) -
        ((b_ct_idx_181_tmp * t293 * 2.0 + c_ct_idx_181_tmp * ct_idx_254 * 4.0) +
         ct_idx_181_tmp * ct_idx_306 * 2.0) *
            t2247) +
       ((ct_idx_181_tmp * t294 * 2.0 + b_ct_idx_181_tmp * ct_idx_252 * 2.0) -
        c_ct_idx_181_tmp * ct_idx_305 * 4.0) *
           t2248) +
      ((c_ct_idx_181_tmp * t292 * 4.0 - ct_idx_181_tmp * ct_idx_253 * 2.0) +
       b_ct_idx_181_tmp * ct_idx_304 * 2.0) *
          t2249;
  b_jacobian_WRh_to_oTg1_oTg2[36] =
      ((((t80 * -2.0 + t89 * 2.0) - ct_idx_258 * t81) + t525 * t85) -
       t3 * 4.0) +
      b_ct_idx_256 * t296;
  b_jacobian_WRh_to_oTg1_oTg2[37] =
      ((((t277 + t59) - t84 * 4.0) - t297 * t85) - t524 * t81) -
      b_ct_idx_256 * ct_idx_256;
  b_jacobian_WRh_to_oTg1_oTg2[38] =
      (((ct_idx_118 + t6) + ct_idx_257 * t85) + t295 * t81) -
      b_ct_idx_256 * t523;
  t84 = ((in21[5] * t1717 -
          (ct_idx_119_tmp * t292 * 2.0 - b_ct_idx_119_tmp * ct_idx_253 * 2.0) *
              ct_idx_381_tmp) -
         (b_ct_idx_119_tmp * t294 * 2.0 - ct_idx_119_tmp * ct_idx_305 * 2.0) *
             ct_idx_390_tmp) +
        (ct_idx_119_tmp * ct_idx_254 * 2.0 +
         b_ct_idx_119_tmp * ct_idx_306 * 2.0) *
            ct_idx_391_tmp;
  t89 = ((-((b_ct_idx_132_tmp * t294 * 4.0 +
             b_ct_idx_183_tmp * ct_idx_252 * 2.0) +
            ct_idx_183_tmp * ct_idx_305 * 2.0) *
              ct_idx_390_tmp +
          in21[3] * ct_idx_513_tmp) +
         ((ct_idx_183_tmp * t292 * 2.0 + b_ct_idx_132_tmp * ct_idx_253 * 4.0) -
          b_ct_idx_183_tmp * ct_idx_304 * 2.0) *
             ct_idx_381_tmp) +
        ((b_ct_idx_183_tmp * t293 * 2.0 - ct_idx_183_tmp * ct_idx_254 * 2.0) +
         b_ct_idx_132_tmp * ct_idx_306 * 4.0) *
            ct_idx_391_tmp;
  b_jacobian_WRh_to_oTg1_oTg2[39] =
      ((((((((((ct_idx_205 - ct_idx_208) + -ct_idx_97 * t7) +
              b_ct_idx_90 * t85) +
             b_ct_idx_96 * t81) -
            t296 * t84) +
           ct_idx_210_tmp * 4.0) +
          ct_idx_192 * ct_idx_258) -
         t2562 * (ct_idx_91 - ct_idx_92)) +
        b_ct_idx_256 * t88) -
       t525 * t89) +
      (ct_idx_96 + ct_idx_104) * t4;
  b_jacobian_WRh_to_oTg1_oTg2[40] =
      ((((((((((-ct_idx_203 + ct_idx_213) + t278) - b_ct_idx_89 * t81) +
             ct_idx_256 * t84) -
            t2562 * (ct_idx_90 + ct_idx_104)) -
           t86 * t85) +
          ct_idx_206_tmp * 4.0) +
         ct_idx_192 * t524) +
        b_ct_idx_94 * b_ct_idx_256) +
       t297 * t89) -
      (ct_idx_91 + ct_idx_93_tmp) * t4;
  b_jacobian_WRh_to_oTg1_oTg2[41] =
      (((((((((ct_idx_209 + ct_idx_212) - b_ct_idx_95 * t85) + t523 * t84) -
            (ct_idx_92 - ct_idx_93_tmp) * t7) +
           t87 * t81) -
          ct_idx_192 * t295) -
         b_ct_idx_88 * b_ct_idx_256) +
        (ct_idx_89 - ct_idx_149_tmp * 4.0) * t4) -
       ct_idx_257 * t89) +
      t2562 * (ct_idx_88 + ct_idx_98_tmp * 4.0);
  t85 = ((((-(t294 * t1779) + ct_idx_253 * t1777) - ct_idx_306 * t1707) +
          t1698 * ct_idx_375) +
         t1731 * ct_idx_380) +
        b_ct_idx_254 * t2248_tmp;
  b_jacobian_WRh_to_oTg1_oTg2[42] =
      (-t299 * ct_idx_226 - ct_idx_222 * t528) + ct_idx_261 * t85;
  b_jacobian_WRh_to_oTg1_oTg2[43] =
      (ct_idx_222 * t300 + ct_idx_226 * ct_idx_259) + t527 * t85;
  b_jacobian_WRh_to_oTg1_oTg2[44] =
      (-ct_idx_260 * ct_idx_222 + ct_idx_226 * t526) - t298 * t85;
  t88 = in19[2] * ct_idx_259 - in19[1] * t526;
  b_jacobian_WRh_to_oTg1_oTg2[45] =
      (-ct_idx_93 * ct_idx_222 - ct_idx_226 * t88) - b_ct_idx_99 * t85;
  t86 = in19[0] * ct_idx_260 - in19[2] * t528;
  b_jacobian_WRh_to_oTg1_oTg2[46] =
      ((((-t135 - t1556) + t298 * t2721_tmp) - b_ct_idx_97 * ct_idx_226) +
       ct_idx_222 * t86) +
      b_ct_idx_92 * t85;
  t87 = in19[1] * ct_idx_261 - in19[0] * t527;
  b_jacobian_WRh_to_oTg1_oTg2[47] =
      ((((-ct_idx_190 + t136) + t527 * t2721_tmp) + b_ct_idx_91 * ct_idx_226) +
       b_ct_idx_98 * ct_idx_222) -
      t87 * t85;
  t85 = ((((-(t292 * t1780) + ct_idx_254 * t1781) - ct_idx_305 * t1708) +
          t1699 * ct_idx_376) +
         t1735 * ct_idx_378) +
        ct_idx_255 * t2249_tmp;
  b_jacobian_WRh_to_oTg1_oTg2[48] =
      (-ct_idx_261 * ct_idx_224 + t289 * t528) - t299 * t85;
  b_jacobian_WRh_to_oTg1_oTg2[49] =
      (-t300 * t289 - ct_idx_224 * t527) + ct_idx_259 * t85;
  b_jacobian_WRh_to_oTg1_oTg2[50] =
      (ct_idx_224 * t298 + t289 * ct_idx_260) + t526 * t85;
  b_jacobian_WRh_to_oTg1_oTg2[51] =
      ((((-t298 * t2721_tmp + t135) + t1556) + ct_idx_93 * t289) +
       b_ct_idx_99 * ct_idx_224) -
      t88 * t85;
  b_jacobian_WRh_to_oTg1_oTg2[52] =
      (-b_ct_idx_92 * ct_idx_224 - t289 * t86) - b_ct_idx_97 * t85;
  b_jacobian_WRh_to_oTg1_oTg2[53] =
      ((((t138 + -ct_idx_261 * t2721_tmp) - t1785) - b_ct_idx_98 * t289) +
       ct_idx_224 * t87) +
      b_ct_idx_91 * t85;
  t85 = ((((-(t293 * t1778) + ct_idx_252 * t1776) - ct_idx_304 * t1706) +
          t1697 * ct_idx_377) +
         t1730 * ct_idx_379) +
        b_ct_idx_253 * t2247_tmp;
  b_jacobian_WRh_to_oTg1_oTg2[54] =
      (ct_idx_225 * t299 + ct_idx_223 * ct_idx_261) + t528 * t85;
  b_jacobian_WRh_to_oTg1_oTg2[55] =
      (-ct_idx_259 * ct_idx_225 + ct_idx_223 * t527) - t300 * t85;
  b_jacobian_WRh_to_oTg1_oTg2[56] =
      (-t298 * ct_idx_223 - ct_idx_225 * t526) + ct_idx_260 * t85;
  b_jacobian_WRh_to_oTg1_oTg2[57] =
      ((((ct_idx_190 - t136) + -t527 * t2721_tmp) - b_ct_idx_99 * ct_idx_223) +
       ct_idx_225 * t88) +
      ct_idx_93 * t85;
  b_jacobian_WRh_to_oTg1_oTg2[58] =
      ((((-t138 + t1785) + ct_idx_261 * t2721_tmp) + b_ct_idx_92 * ct_idx_223) +
       b_ct_idx_97 * ct_idx_225) -
      t86 * t85;
  b_jacobian_WRh_to_oTg1_oTg2[59] =
      (-b_ct_idx_91 * ct_idx_225 - ct_idx_223 * t87) - b_ct_idx_98 * t85;
  t85 =
      (((((ct_idx_245_tmp * t293 * 2.0 + b_ct_idx_245_tmp * ct_idx_254 * 2.0) *
              t2485 -
          (ct_idx_245_tmp * ct_idx_304 * 2.0 + b_ct_idx_245_tmp * t292 * 2.0) *
              t2483) +
         -t2484 * (ct_idx_245_tmp * ct_idx_252 * 2.0 -
                   b_ct_idx_245_tmp * ct_idx_305 * 2.0)) -
        (ct_idx_679_tmp * t293 * 2.0 + ct_idx_55_tmp * ct_idx_254 * 2.0) *
            t2598) +
       (ct_idx_55_tmp * t292 * 2.0 + ct_idx_679_tmp * ct_idx_304 * 2.0) *
           t2600) +
      t2599 * (ct_idx_679_tmp * ct_idx_252 * 2.0 -
               ct_idx_55_tmp * ct_idx_305 * 2.0);
  t81 =
      (((((ct_idx_246_tmp * t292 * 2.0 + b_ct_idx_246_tmp * ct_idx_253 * 2.0) *
              t2483 -
          (ct_idx_246_tmp * ct_idx_305 * 2.0 + b_ct_idx_246_tmp * t294 * 2.0) *
              t2484) +
         -t2485 * (ct_idx_246_tmp * ct_idx_254 * 2.0 -
                   b_ct_idx_246_tmp * ct_idx_306 * 2.0)) -
        (ct_idx_690_tmp * t292 * 2.0 + ct_idx_59_tmp * ct_idx_253 * 2.0) *
            t2600) +
       (ct_idx_59_tmp * t294 * 2.0 + ct_idx_690_tmp * ct_idx_305 * 2.0) *
           t2599) +
      t2598 * (ct_idx_690_tmp * ct_idx_254 * 2.0 -
               ct_idx_59_tmp * ct_idx_306 * 2.0);
  b_jacobian_WRh_to_oTg1_oTg2[60] =
      (((-ct_idx_188 + t2723_tmp * 2.0) - t92 * ct_idx_261) + t299 * t81) -
      t528 * t85;
  b_jacobian_WRh_to_oTg1_oTg2[61] =
      (((ct_idx_138 - t146) - t92 * t527) + t300 * t85) - ct_idx_259 * t81;
  b_jacobian_WRh_to_oTg1_oTg2[62] =
      (((t144 + t2721) + t92 * t298) - ct_idx_260 * t85) - t526 * t81;
  t84 = ((in23[4] * t2228 -
          (ct_idx_447_tmp * t294 * 2.0 + ct_idx_39_tmp * ct_idx_252 * 2.0) *
              t2420) +
         (ct_idx_39_tmp * t293 * 2.0 + ct_idx_447_tmp * ct_idx_306 * 2.0) *
             t2421) +
        t2419 * (ct_idx_447_tmp * ct_idx_253 * 2.0 -
                 ct_idx_39_tmp * ct_idx_304 * 2.0);
  t89 = in20[3] * ct_idx_463;
  b_jacobian_WRh_to_oTg1_oTg2[63] =
      (((((((((t91 + in19[1] * t2721) + ct_idx_165) +
             t2713 * (ct_idx_131 + ct_idx_135)) -
            ct_idx_249 * t299) +
           ct_idx_250 * t528) +
          b_ct_idx_99 * t92) +
         ct_idx_261 * t84) -
        ct_idx_93 * t85) +
       t89 * 2.0) +
      t88 * t81;
  t80 = in20[1] * t90;
  b_jacobian_WRh_to_oTg1_oTg2[64] =
      (((((((((-t1709 + ct_idx_164) - ct_idx_132 * t2715) - ct_idx_250 * t300) +
            ct_idx_249 * ct_idx_259) -
           b_ct_idx_92 * t92) +
          t527 * t84) +
         b_ct_idx_97 * t81) +
        t80 * 2.0) +
       (ct_idx_107 + ct_idx_135) * t2721_tmp) +
      t86 * t85;
  b_jacobian_WRh_to_oTg1_oTg2[65] = (((((((((ct_idx_721 + t1558) + ct_idx_189) +
                                           t2715 * (ct_idx_107 + ct_idx_131)) +
                                          ct_idx_150 * t2713) +
                                         ct_idx_250 * ct_idx_260) +
                                        ct_idx_249 * t526) -
                                       t298 * t84) +
                                      t92 * t87) -
                                     b_ct_idx_91 * t81) +
                                    b_ct_idx_98 * t85;
  t85 =
      ((((((ct_idx_259_tmp * t294 * 2.0 + ct_idx_246_tmp * ct_idx_252 * 4.0) +
           b_ct_idx_259_tmp * ct_idx_305 * 2.0) *
              t2484 -
          ((b_ct_idx_259_tmp * t292 * 2.0 + ct_idx_259_tmp * ct_idx_253 * 2.0) -
           ct_idx_246_tmp * ct_idx_304 * 4.0) *
              t2483) -
         ((ct_idx_246_tmp * t293 * 4.0 - b_ct_idx_259_tmp * ct_idx_254 * 2.0) +
          ct_idx_259_tmp * ct_idx_306 * 2.0) *
             t2485) -
        ((ct_idx_300 * t294 * 2.0 + ct_idx_690_tmp * ct_idx_252 * 4.0) +
         ct_idx_251 * ct_idx_305 * 2.0) *
            t2599) +
       ((ct_idx_251 * t292 * 2.0 + ct_idx_300 * ct_idx_253 * 2.0) -
        ct_idx_690_tmp * ct_idx_304 * 4.0) *
           t2600) +
      ((ct_idx_690_tmp * t293 * 4.0 - ct_idx_251 * ct_idx_254 * 2.0) +
       ct_idx_300 * ct_idx_306 * 2.0) *
          t2598;
  b_jacobian_WRh_to_oTg1_oTg2[66] =
      (((t147 - ct_idx_163) - t299 * t85) - t97 * ct_idx_261) - t93 * t528;
  b_jacobian_WRh_to_oTg1_oTg2[67] =
      ((((-t144 + t148) + b_t2721_tmp * 4.0) + ct_idx_259 * t85) + t93 * t300) -
      t97 * t527;
  t81 = in20[0] * t2721_tmp;
  b_jacobian_WRh_to_oTg1_oTg2[68] =
      ((((t81 * -2.0 - t146) + t526 * t85) + t149 * 4.0) + t97 * t298) -
      t93 * ct_idx_260;
  t84 =
      ((in23[3] * t2230 - (ct_idx_431_tmp * ct_idx_305 * 2.0 +
                           ct_idx_440_tmp * ct_idx_252 * 2.0) *
                              t2420) +
       t2421 *
           (ct_idx_440_tmp * t293 * 2.0 - ct_idx_431_tmp * ct_idx_254 * 2.0)) +
      t2419 * (ct_idx_431_tmp * t292 * 2.0 - ct_idx_440_tmp * ct_idx_304 * 2.0);
  b_jacobian_WRh_to_oTg1_oTg2[69] =
      (((((((((ct_idx_462 + t94) - t88 * t85) + ct_idx_117 * t299) +
            t5 * ct_idx_261) -
           ct_idx_93 * t93) +
          b_ct_idx_99 * t97) +
         t528 * t84) +
        t2713 * (ct_idx_132 - ct_idx_133)) -
       (ct_idx_106 + ct_idx_130_tmp_tmp * 4.0) * t2721_tmp) -
      t2715 * (ct_idx_108 - ct_idx_109_tmp * 4.0);
  t79 = in20[0] * t90;
  b_jacobian_WRh_to_oTg1_oTg2[70] =
      ((((((((((-ct_idx_721 + in19[0] * t146) - b_ct_idx_97 * t85) -
              t2715 * (ct_idx_122 + ct_idx_135)) +
             (ct_idx_105 - ct_idx_132) * t2721_tmp) -
            ct_idx_117 * ct_idx_259) +
           t5 * t527) -
          b_ct_idx_92 * t97) -
         t300 * t84) +
        t93 * t86) +
       t1557 * 4.0) +
      t79 * 2.0;
  t83 = in20[0] * ct_idx_463;
  b_jacobian_WRh_to_oTg1_oTg2[71] =
      ((((((((((t83 * -2.0 - t1709) + ct_idx_164) + b_ct_idx_91 * t85) +
             t2715 * (ct_idx_105 + ct_idx_133)) -
            t5 * t298) -
           ct_idx_117 * t526) +
          b_ct_idx_98 * t93) +
         ct_idx_260 * t84) +
        t97 * t87) +
       t80 * 4.0) +
      (ct_idx_122 + ct_idx_131) * t2721_tmp;
  t85 =
      ((((((b_ct_idx_246_tmp * t293 * 4.0 + ct_idx_259_tmp * ct_idx_254 * 2.0) +
           b_ct_idx_259_tmp * ct_idx_306 * 2.0) *
              t2485 -
          ((ct_idx_259_tmp * t292 * 2.0 - b_ct_idx_259_tmp * ct_idx_253 * 2.0) +
           b_ct_idx_246_tmp * ct_idx_304 * 4.0) *
              t2483) -
         ((b_ct_idx_259_tmp * t294 * 2.0 +
           b_ct_idx_246_tmp * ct_idx_252 * 4.0) -
          ct_idx_259_tmp * ct_idx_305 * 2.0) *
             t2484) -
        ((ct_idx_59_tmp * t293 * 4.0 + ct_idx_300 * ct_idx_254 * 2.0) +
         ct_idx_251 * ct_idx_306 * 2.0) *
            t2598) +
       ((ct_idx_300 * t292 * 2.0 - ct_idx_251 * ct_idx_253 * 2.0) +
        ct_idx_59_tmp * ct_idx_304 * 4.0) *
           t2600) +
      ((ct_idx_251 * t294 * 2.0 + ct_idx_59_tmp * ct_idx_252 * 4.0) -
       ct_idx_300 * ct_idx_305 * 2.0) *
          t2599;
  t84 =
      (((((ct_idx_79_tmp * t294 * 2.0 - ct_idx_419_tmp * ct_idx_252 * 2.0) *
              t2484 -
          (ct_idx_419_tmp * ct_idx_304 * 2.0 +
           ct_idx_79_tmp * ct_idx_253 * 2.0) *
              t2483) -
         (ct_idx_119 * t294 * 2.0 - t279 * ct_idx_252 * 2.0) * t2599) +
        (t279 * ct_idx_304 * 2.0 + ct_idx_119 * ct_idx_253 * 2.0) * t2600) +
       -t2598 * (t279 * t293 * 2.0 - ct_idx_119 * ct_idx_306 * 2.0)) +
      t2485 * (ct_idx_419_tmp * t293 * 2.0 - ct_idx_79_tmp * ct_idx_306 * 2.0);
  b_jacobian_WRh_to_oTg1_oTg2[72] =
      ((((t2721 - t148) + t299 * t85) + t99 * 4.0) + t101 * t528) +
      ct_idx_261 * t84;
  b_jacobian_WRh_to_oTg1_oTg2[73] =
      (((-t98 - ct_idx_163) - ct_idx_259 * t85) - t101 * t300) + t527 * t84;
  b_jacobian_WRh_to_oTg1_oTg2[74] =
      ((((t100 + t2723) - t526 * t85) + ct_idx_160 * 4.0) + t101 * ct_idx_260) -
      t298 * t84;
  t78 =
      ((in23[4] * t2230 +
        (ct_idx_39_tmp * t294 * 2.0 - ct_idx_447_tmp * ct_idx_252 * 2.0) *
            t2420) -
       (ct_idx_447_tmp * ct_idx_304 * 2.0 + ct_idx_39_tmp * ct_idx_253 * 2.0) *
           t2419) +
      t2421 * (ct_idx_447_tmp * t293 * 2.0 - ct_idx_39_tmp * ct_idx_306 * 2.0);
  b_jacobian_WRh_to_oTg1_oTg2[75] =
      ((((((((((t1558 + t79 * -2.0) + ct_idx_189) +
              t2713 * (ct_idx_106 + ct_idx_130_tmp)) +
             t2715 * (ct_idx_134 + ct_idx_135)) +
            t88 * t85) +
           ct_idx_722 * 4.0) -
          t1710 * t299) -
         t290 * t528) +
        ct_idx_93 * t101) +
       ct_idx_261 * t78) -
      b_ct_idx_99 * t84;
  b_jacobian_WRh_to_oTg1_oTg2[76] =
      (((((((((t94 - ct_idx_294) + b_ct_idx_97 * t85) + t290 * t300) +
            t1710 * ct_idx_259) +
           t527 * t78) -
          t101 * t86) +
         b_ct_idx_92 * t84) -
        (ct_idx_130_tmp + ct_idx_150) * t2721_tmp) -
       t2713 * (ct_idx_105 - ct_idx_132_tmp * 4.0)) -
      t2715 * (ct_idx_108 + ct_idx_111_tmp_tmp * 4.0);
  b_jacobian_WRh_to_oTg1_oTg2[77] =
      ((((((((((t1711 + t89 * -2.0) + ct_idx_109 * t2721_tmp) -
              b_ct_idx_91 * t85) -
             t2713 * (ct_idx_107 + ct_idx_134)) +
            t2715 * (ct_idx_106 + ct_idx_150)) -
           t290 * ct_idx_260) +
          t1710 * t526) -
         b_ct_idx_98 * t101) -
        t298 * t78) +
       t95 * 4.0) -
      t87 * t84;
  t85 =
      ((((((b_ct_idx_245_tmp * t294 * 4.0 +
            b_ct_idx_257_tmp * ct_idx_252 * 2.0) +
           ct_idx_257_tmp * ct_idx_305 * 2.0) *
              t2484 -
          ((ct_idx_257_tmp * t292 * 2.0 + b_ct_idx_245_tmp * ct_idx_253 * 4.0) -
           b_ct_idx_257_tmp * ct_idx_304 * 2.0) *
              t2483) -
         ((b_ct_idx_257_tmp * t293 * 2.0 - ct_idx_257_tmp * ct_idx_254 * 2.0) +
          b_ct_idx_245_tmp * ct_idx_306 * 4.0) *
             t2485) -
        ((ct_idx_55_tmp * t294 * 4.0 + ct_idx_168 * ct_idx_252 * 2.0) +
         ct_idx_166 * ct_idx_305 * 2.0) *
            t2599) +
       ((ct_idx_166 * t292 * 2.0 + ct_idx_55_tmp * ct_idx_253 * 4.0) -
        ct_idx_168 * ct_idx_304 * 2.0) *
           t2600) +
      ((ct_idx_168 * t293 * 2.0 - ct_idx_166 * ct_idx_254 * 2.0) +
       ct_idx_55_tmp * ct_idx_306 * 4.0) *
          t2598;
  t84 =
      ((((((b_ct_idx_258_tmp * t293 * 2.0 + ct_idx_419_tmp * ct_idx_254 * 4.0) +
           ct_idx_258_tmp * ct_idx_306 * 2.0) *
              t2485 -
          ((ct_idx_419_tmp * t292 * 4.0 - ct_idx_258_tmp * ct_idx_253 * 2.0) +
           b_ct_idx_258_tmp * ct_idx_304 * 2.0) *
              t2483) -
         ((ct_idx_258_tmp * t294 * 2.0 + b_ct_idx_258_tmp * ct_idx_252 * 2.0) -
          ct_idx_419_tmp * ct_idx_305 * 4.0) *
             t2484) -
        ((ct_idx_301 * t293 * 2.0 + t279 * ct_idx_254 * 4.0) +
         ct_idx_302 * ct_idx_306 * 2.0) *
            t2598) +
       ((t279 * t292 * 4.0 - ct_idx_302 * ct_idx_253 * 2.0) +
        ct_idx_301 * ct_idx_304 * 2.0) *
           t2600) +
      ((ct_idx_302 * t294 * 2.0 + ct_idx_301 * ct_idx_252 * 2.0) -
       t279 * ct_idx_305 * 4.0) *
          t2599;
  b_jacobian_WRh_to_oTg1_oTg2[78] =
      ((((-ct_idx_138 + t81 * 2.0) + ct_idx_261 * t84) - t528 * t85) +
       t145 * 4.0) +
      t96 * t299;
  b_jacobian_WRh_to_oTg1_oTg2[79] =
      ((((-t100 - ct_idx_188) + t2723_tmp * 4.0) + t300 * t85) + t527 * t84) -
      t96 * ct_idx_259;
  b_jacobian_WRh_to_oTg1_oTg2[80] =
      (((t147 - t98) - ct_idx_260 * t85) - t298 * t84) - t96 * t526;
  t81 =
      ((in23[5] * t2230 - (ct_idx_444_tmp * ct_idx_306 * 2.0 +
                           ct_idx_449_tmp * ct_idx_254 * 2.0) *
                              t2421) +
       t2419 *
           (ct_idx_449_tmp * t292 * 2.0 - ct_idx_444_tmp * ct_idx_253 * 2.0)) +
      t2420 * (ct_idx_444_tmp * t294 * 2.0 - ct_idx_449_tmp * ct_idx_305 * 2.0);
  b_jacobian_WRh_to_oTg1_oTg2[81] =
      ((((((((((t80 * -2.0 + in19[2] * ct_idx_188) - ct_idx_93 * t85) -
              b_ct_idx_99 * t84) +
             ct_idx_960 * 4.0) -
            t274 * ct_idx_261) +
           t139 * t528) -
          t299 * t81) +
         t2713 * (ct_idx_108 - ct_idx_109)) +
        t96 * t88) +
       t83 * 2.0) -
      (ct_idx_131 + ct_idx_136) * t2721_tmp;
  b_jacobian_WRh_to_oTg1_oTg2[82] =
      ((((((((((-t1711 + t91) + ct_idx_165) + b_ct_idx_92 * t84) +
             t2713 * (ct_idx_107 + ct_idx_136)) +
            t86 * t85) -
           t139 * t300) -
          t274 * t527) +
         b_ct_idx_97 * t96) +
        ct_idx_259 * t81) +
       t89 * 4.0) +
      (ct_idx_108 + ct_idx_111_tmp) * t2721_tmp;
  b_jacobian_WRh_to_oTg1_oTg2[83] =
      (((((((((ct_idx_462 - ct_idx_294) + b_ct_idx_98 * t85) - t87 * t84) +
            t274 * t298) +
           t139 * ct_idx_260) -
          b_ct_idx_91 * t96) +
         t526 * t81) +
        t2715 * (ct_idx_109 - ct_idx_111_tmp)) -
       (ct_idx_106 - ct_idx_150_tmp * 4.0) * t2721_tmp) -
      t2713 * (ct_idx_105 + ct_idx_133_tmp * 4.0);
}

//
// File trailer for jacobian_WRh_to_oTg1_oTg2.cpp
//
// [EOF]
//
