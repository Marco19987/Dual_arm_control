//
// Academic License - for use in teaching, academic research, and meeting
// course requirements at degree granting institutions only.  Not for
// government, commercial, or other organizational use.
// File: jacobian_h_to_b2Tb1.cpp
//
// MATLAB Coder version            : 23.2
// C/C++ source code generated on  : 11-Apr-2025 10:25:57
//

// Include Files
#include "jacobian_h_to_b2Tb1.h"
#include "rt_nonfinite.h"
#include "rt_nonfinite.cpp"
#include <cmath>
#include <cstring>

// Function Declarations
static void ft_1(const double ct[343], double b_jacobian_h_to_b2Tb1[84]);

static double rt_powd_snf(double u0, double u1);

// Function Definitions
//
// Arguments    : const double ct[343]
//                double b_jacobian_h_to_b2Tb1[84]
// Return Type  : void
//
static void ft_1(const double ct[343], double b_jacobian_h_to_b2Tb1[84])
{
  double ct_idx_100;
  double ct_idx_101;
  double ct_idx_102;
  double ct_idx_103;
  double ct_idx_104;
  double ct_idx_105;
  double ct_idx_106;
  double ct_idx_270;
  double ct_idx_271;
  double ct_idx_272;
  double ct_idx_28_tmp;
  double ct_idx_29_tmp;
  double ct_idx_309_tmp;
  double ct_idx_315_tmp;
  double ct_idx_316_tmp;
  double ct_idx_325_tmp;
  double ct_idx_326_tmp;
  double ct_idx_332_tmp;
  double ct_idx_337_tmp;
  double ct_idx_351_tmp;
  double ct_idx_352_tmp;
  double ct_idx_35_tmp;
  double ct_idx_360_tmp;
  double ct_idx_361_tmp;
  double ct_idx_36_tmp;
  double ct_idx_40_tmp;
  double ct_idx_44_tmp;
  double ct_idx_76;
  double ct_idx_77;
  double ct_idx_78;
  double ct_idx_79;
  double ct_idx_80;
  double ct_idx_81;
  double ct_idx_82;
  double ct_idx_83;
  double ct_idx_84;
  double ct_idx_89;
  double ct_idx_90;
  double ct_idx_91;
  double ct_idx_92;
  double ct_idx_93;
  double ct_idx_94;
  double ct_idx_95;
  double ct_idx_96;
  double ct_idx_97;
  double ct_idx_98;
  double ct_idx_99;
  double t1116_tmp;
  double t1120_tmp;
  double t1124_tmp;
  double t1134;
  double t1135;
  double t1136;
  double t1137;
  double t1138;
  double t1139;
  double t1140;
  double t1141;
  double t1142;
  double t1143;
  double t1144;
  double t1145;
  double t1146;
  double t1147;
  double t1148;
  double t1149;
  double t1150;
  double t1151;
  double t1176;
  double t1177;
  double t1178;
  double t1181;
  double t1183;
  double t1184;
  double t1186;
  double t1187;
  double t1216;
  double t1231;
  double t1233;
  double t1238;
  double t1269;
  double t1270;
  double t1271;
  double t1284;
  double t1285;
  double t1286;
  double t1287;
  double t1288;
  double t1289;
  double t1329;
  double t1330;
  double t1331;
  double t1334;
  double t1339;
  double t1340;
  double t1341;
  double t1342;
  double t1343;
  double t1344;
  double t1345;
  double t1346;
  double t1347;
  double t1348;
  double t1349;
  double t1350;
  double t1351;
  double t1352;
  double t1353;
  double t1354;
  double t1355;
  double t1357;
  double t1359;
  double t553;
  double t554;
  double t555;
  double t556;
  double t557;
  double t558;
  double t583;
  double t584;
  double t585;
  double t586;
  double t587;
  double t588;
  double t589;
  double t590;
  double t591;
  double t592;
  double t593;
  double t594;
  double t595;
  double t596;
  double t597;
  double t629;
  double t630;
  double t631;
  double t632;
  double t633;
  double t634;
  double t635;
  double t636;
  double t637;
  double t638;
  double t639;
  double t640;
  double t641;
  double t642;
  double t643;
  double t644;
  double t658;
  double t659;
  t1355 = (ct[338] + ct[341]) + ct[78];
  t553 = (t1355 + ct[80]) + ct[100];
  t554 = (t1355 + ct[84]) + ct[96];
  t1355 = (ct[339] + ct[342]) + ct[79];
  t555 = (t1355 + ct[82]) + ct[104];
  t556 = (t1355 + ct[89]) + ct[98];
  t1355 = (ct[32] + ct[340]) + ct[81];
  t557 = (t1355 + ct[85]) + ct[105];
  t558 = (t1355 + ct[91]) + ct[102];
  t583 = ((((ct[41] + ct[254]) + ct[83]) + ct[87]) + ct[146]) + ct[150];
  t584 = ((((ct[44] + ct[159]) + ct[88]) + ct[90]) + ct[147]) + ct[151];
  t585 = ((((ct[46] + ct[86]) + ct[93]) + ct[95]) + ct[149]) + ct[153];
  t586 = ((((ct[43] + ct[323]) + ct[92]) + ct[94]) + ct[148]) + ct[155];
  t587 = ((((ct[45] + ct[315]) + ct[97]) + ct[99]) + ct[152]) + ct[156];
  t588 = ((((ct[47] + ct[309]) + ct[101]) + ct[103]) + ct[154]) + ct[157];
  t589 = ((((ct[40] + ct[41]) + ct[83]) + ct[87]) + ct[146]) + ct[150];
  t590 = ((((ct[38] + ct[44]) + ct[88]) + ct[90]) + ct[147]) + ct[151];
  t591 = ((((ct[36] + ct[46]) + ct[93]) + ct[95]) + ct[149]) + ct[153];
  t592 = ((((ct[42] + ct[43]) + ct[92]) + ct[94]) + ct[148]) + ct[155];
  t593 = ((((ct[39] + ct[45]) + ct[97]) + ct[99]) + ct[152]) + ct[156];
  t594 = ((((ct[37] + ct[47]) + ct[101]) + ct[103]) + ct[154]) + ct[157];
  t595 = ((((((ct[33] + ct[70]) + ct[71]) + ct[72]) + ct[217]) + ct[220]) +
          ct[221]) +
         1.0;
  t596 = ((((((ct[34] + ct[69]) + ct[72]) + ct[73]) + ct[218]) + ct[220]) +
          ct[222]) +
         1.0;
  t597 = ((((((ct[35] + ct[69]) + ct[70]) + ct[74]) + ct[219]) + ct[221]) +
          ct[222]) +
         1.0;
  t658 = 1.0 / ct[337];
  t659 = t658 * t658;
  t1349 = ct[274] * ct[333];
  t1346 = ct[272] * ct[334];
  t1345 = ct[273] * ct[335];
  ct_idx_28_tmp = ct[287] * ct[333];
  ct_idx_29_tmp = ct[289] * ct[334];
  ct_idx_35_tmp = ct[288] * ct[335];
  ct_idx_36_tmp = ct[25] * ct[333];
  ct_idx_40_tmp = ct[26] * ct[334];
  ct_idx_44_tmp = ct[27] * ct[335];
  ct_idx_76 = (ct[313] + ct[0] * ct[276] * ct[295]) + ct[0] * ct[279] * ct[290];
  ct_idx_77 = (ct[314] + ct[1] * ct[277] * ct[294]) + ct[1] * ct[280] * ct[292];
  ct_idx_78 = (ct[316] + ct[2] * ct[275] * ct[293]) + ct[2] * ct[278] * ct[291];
  ct_idx_79 = (ct[317] + ct[3] * ct[276] * ct[295]) + ct[3] * ct[279] * ct[290];
  ct_idx_80 = (ct[318] + ct[4] * ct[277] * ct[294]) + ct[4] * ct[280] * ct[292];
  ct_idx_81 = (ct[319] + ct[5] * ct[275] * ct[293]) + ct[5] * ct[278] * ct[291];
  ct_idx_82 = (ct[320] + ct[6] * ct[276] * ct[295]) + ct[6] * ct[279] * ct[290];
  ct_idx_83 = (ct[321] + ct[7] * ct[277] * ct[294]) + ct[7] * ct[280] * ct[292];
  ct_idx_84 = (ct[322] + ct[8] * ct[275] * ct[293]) + ct[8] * ct[278] * ct[291];
  t1354 = ((ct[297] + ct[304]) + ct[311]) - ct[307];
  t1340 = ((ct[298] + ct[303]) + ct[308]) - ct[310];
  t1339 = ((ct[299] + ct[305]) + ct[31] * ct[296]) - ct[302];
  t1334 = ((ct[301] + ct[306]) + ct[312]) - ct[300];
  t1355 = ct[0] * ct[271];
  ct_idx_89 =
      (ct[0] * ct[268] * ct[295] + t1355 * ct[291]) - ct[0] * ct[275] * ct[279];
  t1353 = ct[1] * ct[269];
  ct_idx_90 =
      (ct[1] * ct[266] * ct[294] + t1353 * ct[290]) - ct[1] * ct[276] * ct[280];
  ct_idx_91 =
      (ct[0] * ct[267] * ct[279] + t1355 * ct[277]) - ct[0] * ct[292] * ct[295];
  ct_idx_92 =
      (t1353 * ct[275] + ct[1] * ct[268] * ct[280]) - ct[1] * ct[291] * ct[294];
  t1355 = ct[2] * ct[270];
  ct_idx_93 =
      (ct[2] * ct[266] * ct[278] + t1355 * ct[276]) - ct[2] * ct[290] * ct[293];
  ct_idx_94 =
      (ct[2] * ct[267] * ct[293] + t1355 * ct[292]) - ct[2] * ct[277] * ct[278];
  t1355 = ct[3] * ct[271];
  ct_idx_95 =
      (ct[3] * ct[268] * ct[295] + t1355 * ct[291]) - ct[3] * ct[275] * ct[279];
  t1353 = ct[4] * ct[269];
  ct_idx_96 =
      (ct[4] * ct[266] * ct[294] + t1353 * ct[290]) - ct[4] * ct[276] * ct[280];
  ct_idx_97 =
      (ct[3] * ct[267] * ct[279] + t1355 * ct[277]) - ct[3] * ct[292] * ct[295];
  ct_idx_98 =
      (t1353 * ct[275] + ct[4] * ct[268] * ct[280]) - ct[4] * ct[291] * ct[294];
  t1355 = ct[5] * ct[270];
  ct_idx_99 =
      (ct[5] * ct[266] * ct[278] + t1355 * ct[276]) - ct[5] * ct[290] * ct[293];
  ct_idx_100 =
      (ct[5] * ct[267] * ct[293] + t1355 * ct[292]) - ct[5] * ct[277] * ct[278];
  t1355 = ct[6] * ct[271];
  ct_idx_101 =
      (ct[6] * ct[268] * ct[295] + t1355 * ct[291]) - ct[6] * ct[275] * ct[279];
  t1353 = ct[7] * ct[269];
  ct_idx_102 =
      (ct[7] * ct[266] * ct[294] + t1353 * ct[290]) - ct[7] * ct[276] * ct[280];
  ct_idx_103 =
      (ct[6] * ct[267] * ct[279] + t1355 * ct[277]) - ct[6] * ct[292] * ct[295];
  ct_idx_104 =
      (t1353 * ct[275] + ct[7] * ct[268] * ct[280]) - ct[7] * ct[291] * ct[294];
  t1355 = ct[8] * ct[270];
  ct_idx_105 =
      (ct[8] * ct[266] * ct[278] + t1355 * ct[276]) - ct[8] * ct[290] * ct[293];
  ct_idx_106 =
      (ct[8] * ct[267] * ct[293] + t1355 * ct[292]) - ct[8] * ct[277] * ct[278];
  ct_idx_270 =
      ((((((((((((((((((ct[22] + ct[68]) + ct[108]) + ct[121]) + ct[122]) +
                    ct[124]) +
                   ct[135]) +
                  ct[136]) +
                 ct[138]) +
                ct[173]) +
               ct[180]) +
              ct[185]) +
             ct[187]) +
            ct[206]) +
           ct[210]) +
          ct[212]) +
         ct[215]) +
        ct[253]) +
       ct[256]) +
      ct[258];
  ct_idx_271 =
      ((((((((((((((((((ct[23] + ct[67]) + ct[114]) + ct[119]) + ct[120]) +
                    ct[126]) +
                   ct[134]) +
                  ct[140]) +
                 ct[141]) +
                ct[166]) +
               ct[171]) +
              ct[190]) +
             ct[191]) +
            ct[205]) +
           ct[208]) +
          ct[213]) +
         ct[216]) +
        ct[255]) +
       ct[257]) +
      ct[261];
  ct_idx_272 =
      ((((((((((((((((((ct[24] + ct[66]) + ct[117]) + ct[118]) + ct[123]) +
                    ct[125]) +
                   ct[137]) +
                  ct[139]) +
                 ct[142]) +
                ct[175]) +
               ct[181]) +
              ct[186]) +
             ct[189]) +
            ct[207]) +
           ct[209]) +
          ct[211]) +
         ct[214]) +
        ct[259]) +
       ct[260]) +
      ct[262];
  ct_idx_309_tmp = ct[26] * ct[333];
  ct_idx_315_tmp = ct[25] * ct[335];
  ct_idx_316_tmp = ct[27] * ct[334];
  ct_idx_325_tmp = ct[25] * ct[334];
  ct_idx_326_tmp = ct[27] * ct[333];
  ct_idx_332_tmp = ct[26] * ct[335];
  ct_idx_337_tmp = ct[264] * ct[333];
  t1357 = ct[265] * ct[334];
  t1351 = ct[263] * ct[335];
  t1348 = ct[273] * ct[333];
  ct_idx_351_tmp = ct[274] * ct[334];
  ct_idx_352_tmp = ct[272] * ct[335];
  t1347 = ct[263] * ct[333];
  ct_idx_360_tmp = ct[264] * ct[334];
  ct_idx_361_tmp = ct[265] * ct[335];
  t1359 = ct[289] * ct[333];
  t1352 = ct[288] * ct[334];
  t1350 = ct[287] * ct[335];
  t1116_tmp = ct[333] * t659;
  t1120_tmp = ct[334] * t659;
  t1124_tmp = ct[335] * t659;
  t629 = ct[15] * t1354;
  t630 = ct[15] * t1340;
  t631 = ct[15] * t1339;
  t632 = ct[15] * t1334;
  t633 = ct[16] * t1354;
  t634 = ct[16] * t1340;
  t635 = ct[16] * t1339;
  t636 = ct[16] * t1334;
  t637 = ct[17] * t1354;
  t638 = ct[17] * t1340;
  t639 = ct[17] * t1339;
  t640 = ct[17] * t1334;
  t641 = ct[18] * t1354;
  t642 = ct[18] * t1340;
  t643 = ct[18] * t1339;
  t644 = ct[18] * t1334;
  t1134 = (-(ct[263] * t592 * t658) + ct[287] * t587 * t658) +
          ct[273] * t595 * t658;
  t1135 =
      (ct[272] * t587 * t658 + ct[288] * t592 * t658) - ct[265] * t595 * t658;
  t1136 =
      (ct[274] * t592 * t658 - ct[264] * t587 * t658) + ct[289] * t595 * t658;
  t1137 =
      (ct[273] * t586 * t658 + ct[287] * t594 * t658) - ct[263] * t596 * t658;
  t1138 = (-(ct[265] * t586 * t658) + ct[272] * t594 * t658) +
          ct[288] * t596 * t658;
  t1139 = (-(ct[264] * t594 * t658) + ct[289] * t586 * t658) +
          ct[274] * t596 * t658;
  t1140 =
      (ct[273] * t593 * t658 - ct[263] * t588 * t658) + ct[287] * t597 * t658;
  t1141 = (-(ct[265] * t593 * t658) + ct[288] * t588 * t658) +
          ct[272] * t597 * t658;
  t1142 =
      (ct[274] * t588 * t658 + ct[289] * t593 * t658) - ct[264] * t597 * t658;
  t588 = ct[264] * ct[324];
  t593 = ct[289] * ct[324];
  t1329 = ct[274] * ct[324];
  t1143 = ((((ct[129] * ct[264] * t658 * 2.0 + ct[204] * ct[274] * t658 * 2.0) +
             ct[12] * ct[86] * ct[289] * t658 * -8.0) -
            t588 * t584 * t659 * 2.0) +
           t1329 * t589 * t659 * 2.0) +
          t593 * t595 * t659;
  t1340 = ct[272] * ct[324];
  t1339 = ct[288] * ct[324];
  t1334 = ct[265] * ct[324];
  t1144 = ((((ct[130] * ct[265] * t658 * 2.0 + ct[202] * ct[272] * t658 * 2.0) +
             ct[13] * ct[159] * ct[288] * t658 * -8.0) -
            t1334 * t583 * t659 * 2.0) +
           t1340 * t591 * t659 * 2.0) +
          t1339 * t596 * t659;
  t1355 = ct[263] * ct[324];
  t1353 = ct[287] * ct[324];
  t1354 = ct[273] * ct[324];
  t1145 = ((((ct[128] * ct[263] * t658 * 2.0 + ct[203] * ct[273] * t658 * 2.0) +
             ct[14] * ct[254] * ct[287] * t658 * -8.0) -
            t1355 * t585 * t659 * 2.0) +
           t1354 * t590 * t659 * 2.0) +
          t1353 * t597 * t659;
  t1146 = ((((ct[131] * ct[273] * t658 + ct[204] * ct[263] * t658 * 2.0) +
             ct[129] * ct[287] * t658 * 2.0) +
            t1355 * t589 * t659 * 2.0) -
           t1353 * t584 * t659 * 2.0) -
          t1354 * t595 * t659;
  t1147 = ((((ct[131] * ct[265] * t658 - ct[129] * ct[272] * t658 * 2.0) +
             ct[204] * ct[288] * t658 * 2.0) +
            t1340 * t584 * t659 * 2.0) +
           t1339 * t589 * t659 * 2.0) -
          t1334 * t595 * t659;
  t1148 = ((((ct[132] * ct[263] * t658 - ct[130] * ct[273] * t658 * 2.0) +
             ct[202] * ct[287] * t658 * 2.0) +
            t1354 * t583 * t659 * 2.0) +
           t1353 * t591 * t659 * 2.0) -
          t1355 * t596 * t659;
  t1149 = ((((ct[132] * ct[274] * t658 + ct[202] * ct[264] * t658 * 2.0) +
             ct[130] * ct[289] * t658 * 2.0) +
            t588 * t591 * t659 * 2.0) -
           t593 * t583 * t659 * 2.0) -
          t1329 * t596 * t659;
  t1150 = ((((ct[133] * ct[272] * t658 + ct[203] * ct[265] * t658 * 2.0) +
             ct[128] * ct[288] * t658 * 2.0) +
            t1334 * t590 * t659 * 2.0) -
           t1339 * t585 * t659 * 2.0) -
          t1340 * t597 * t659;
  t1151 = ((((ct[133] * ct[264] * t658 - ct[128] * ct[274] * t658 * 2.0) +
             ct[203] * ct[289] * t658 * 2.0) +
            t1329 * t585 * t659 * 2.0) +
           t593 * t590 * t659 * 2.0) -
          t588 * t597 * t659;
  t1355 = ct[265] * ct[333];
  t1353 = ct[288] * ct[333];
  t1354 = ct[272] * ct[333];
  t1176 = ((((ct[201] * ct[272] * t658 * 2.0 + ct[281] * ct[288] * t658) +
             ct[265] * t556 * t658 * 2.0) -
            t1355 * t583 * t659 * 2.0) +
           t1354 * t591 * t659 * 2.0) +
          t1353 * t596 * t659;
  t1340 = ct[263] * ct[334];
  t1339 = ct[273] * ct[334];
  t1334 = ct[287] * ct[334];
  t1177 = ((((-(ct[127] * ct[287] * t658 * 2.0) + ct[273] * ct[283] * t658) +
             ct[263] * t554 * t658 * 2.0) -
            t1340 * t589 * t659 * 2.0) +
           t1334 * t584 * t659 * 2.0) +
          t1339 * t595 * t659;
  t1178 = ((((ct[127] * ct[272] * t658 * 2.0 + ct[265] * ct[283] * t658) +
             ct[288] * t554 * t658 * 2.0) -
            t1346 * t584 * t659 * 2.0) -
           t1352 * t589 * t659 * 2.0) +
          t1357 * t595 * t659;
  t593 = ((((ct[127] * ct[264] * t658 * 2.0 + ct[283] * ct[289] * t658) -
            ct[274] * t554 * t658 * 2.0) -
           ct_idx_360_tmp * t584 * t659 * 2.0) +
          ct_idx_351_tmp * t589 * t659 * 2.0) +
         ct_idx_29_tmp * t595 * t659;
  t588 = ((((ct[127] * ct[263] * t658 * 2.0 + ct[282] * ct[287] * t658) -
            ct[273] * t558 * t658 * 2.0) -
           t1347 * t585 * t659 * 2.0) +
          t1348 * t590 * t659 * 2.0) +
         ct_idx_28_tmp * t597 * t659;
  t1181 = ((((-(ct[127] * ct[288] * t658 * 2.0) + ct[272] * ct[282] * t658) +
             ct[265] * t558 * t658 * 2.0) -
            t1355 * t590 * t659 * 2.0) +
           t1353 * t585 * t659 * 2.0) +
          t1354 * t597 * t659;
  t587 = ((((ct[127] * ct[274] * t658 * 2.0 + ct[264] * ct[282] * t658) +
            ct[289] * t558 * t658 * 2.0) -
           t1349 * t585 * t659 * 2.0) -
          t1359 * t590 * t659 * 2.0) +
         ct_idx_337_tmp * t597 * t659;
  t1329 = ct[264] * ct[335];
  t1330 = ct[289] * ct[335];
  t1331 = ct[274] * ct[335];
  t1183 = ((((ct[201] * ct[274] * t658 * 2.0 + ct[285] * ct[289] * t658) +
             ct[264] * t553 * t658 * 2.0) -
            t1329 * t584 * t659 * 2.0) +
           t1331 * t589 * t659 * 2.0) +
          t1330 * t595 * t659;
  t1184 = ((((ct[201] * ct[273] * t658 * 2.0 + ct[284] * ct[287] * t658) +
             ct[263] * t557 * t658 * 2.0) -
            t1340 * t585 * t659 * 2.0) +
           t1339 * t590 * t659 * 2.0) +
          t1334 * t597 * t659;
  t586 = ((((ct[127] * ct[273] * t658 * 2.0 + ct[263] * ct[286] * t658) +
            ct[287] * t555 * t658 * 2.0) -
           t1345 * t583 * t659 * 2.0) -
          t1350 * t591 * t659 * 2.0) +
         t1351 * t596 * t659;
  t1186 = ((((ct[127] * ct[265] * t658 * 2.0 + ct[286] * ct[288] * t658) -
             ct[272] * t555 * t658 * 2.0) -
            ct_idx_361_tmp * t583 * t659 * 2.0) +
           ct_idx_352_tmp * t591 * t659 * 2.0) +
          ct_idx_35_tmp * t596 * t659;
  t1187 = ((((-(ct[127] * ct[289] * t658 * 2.0) + ct[274] * ct[286] * t658) +
             ct[264] * t555 * t658 * 2.0) -
            t1329 * t591 * t659 * 2.0) +
           t1330 * t583 * t659 * 2.0) +
          t1331 * t596 * t659;
  t592 = ((((ct[201] * ct[264] * t658 * 2.0 - ct[274] * ct[281] * t658) +
            ct[289] * t556 * t658 * 2.0) +
           ct_idx_337_tmp * t591 * t659 * 2.0) -
          t1359 * t583 * t659 * 2.0) -
         t1349 * t596 * t659;
  t594 = ((((ct[201] * ct[263] * t658 * 2.0 - ct[273] * ct[285] * t658) +
            ct[287] * t553 * t658 * 2.0) +
           t1351 * t589 * t659 * 2.0) -
          t1350 * t584 * t659 * 2.0) -
         t1345 * t595 * t659;
  t1216 = ((((ct[201] * ct[265] * t658 * 2.0 - ct[272] * ct[284] * t658) +
             ct[288] * t557 * t658 * 2.0) +
            t1357 * t590 * t659 * 2.0) -
           t1352 * t585 * t659 * 2.0) -
          t1346 * t597 * t659;
  t1231 = ((((ct[272] * t558 * t658 * 2.0 + ct[288] * t556 * t658 * 2.0) -
             ct[265] * ct[329] * t658) -
            t1354 * t584 * t659 * 2.0) -
           t1353 * t589 * t659 * 2.0) +
          t1355 * t595 * t659;
  t1233 = ((((ct[273] * t554 * t658 * 2.0 + ct[287] * t557 * t658 * 2.0) -
             ct[263] * ct[330] * t658) -
            t1339 * t583 * t659 * 2.0) -
           t1334 * t591 * t659 * 2.0) +
          t1340 * t596 * t659;
  t1238 = ((((ct[274] * t555 * t658 * 2.0 + ct[289] * t553 * t658 * 2.0) -
             ct[264] * ct[331] * t658) -
            t1331 * t585 * t659 * 2.0) -
           t1330 * t590 * t659 * 2.0) +
          t1329 * t597 * t659;
  t1355 = ct[27] * ct[324];
  t1353 = ct[25] * ct[324];
  t1354 = ct[26] * ct[324];
  t1340 = ct[324] * t659;
  t1269 = ((((((ct[12] * ct[25] * ct[86] * t658 * -8.0 -
                ct[26] * ct[130] * t658 * 2.0) +
               ct[27] * ct[203] * t658 * 2.0) +
              ((((ct[59] + ct[336]) + ct[193]) + ct[195]) + ct[223]) * t658) +
             t1354 * t583 * t659 * 2.0) +
            t1355 * t590 * t659 * 2.0) +
           t1353 * t595 * t659) -
          t1340 * ct_idx_270;
  t1270 = ((((((ct[13] * ct[26] * ct[159] * t658 * -8.0 +
                ct[25] * ct[204] * t658 * 2.0) -
               ct[27] * ct[128] * t658 * 2.0) +
              ((((ct[56] + ct[332]) + ct[192]) + ct[198]) + ct[225]) * t658) +
             t1355 * t585 * t659 * 2.0) +
            t1353 * t589 * t659 * 2.0) +
           t1354 * t596 * t659) -
          t1340 * ct_idx_271;
  t1271 = ((((((ct[14] * ct[27] * ct[254] * t658 * -8.0 -
                ct[25] * ct[129] * t658 * 2.0) +
               ct[26] * ct[202] * t658 * 2.0) +
              ((((ct[52] + ct[328]) + ct[194]) + ct[196]) + ct[233]) * t658) +
             t1353 * t584 * t659 * 2.0) +
            t1354 * t591 * t659 * 2.0) +
           t1355 * t597 * t659) -
          t1340 * ct_idx_272;
  t1284 =
      ((((((-(ct[27] * ct[127] * t658 * 2.0) + ct[26] * ct[281] * t658) -
           ct[25] * t556 * t658 * 2.0) +
          ((((((((ct[53] + ct[327]) + ct[57]) + ct[109]) + ct[161]) + ct[176]) +
             ct[200]) +
            ct[229]) +
           ct[241]) *
              t658) +
         ct_idx_326_tmp * t585 * t659 * 2.0) +
        ct_idx_36_tmp * t589 * t659 * 2.0) +
       ct_idx_309_tmp * t596 * t659) -
      t1116_tmp * ct_idx_271;
  t1285 =
      ((((((-(ct[26] * ct[127] * t658 * 2.0) + ct[25] * ct[285] * t658) -
           ct[27] * t553 * t658 * 2.0) +
          ((((((((ct[54] + ct[326]) + ct[62]) + ct[110]) + ct[163]) + ct[172]) +
             ct[199]) +
            ct[232]) +
           ct[247]) *
              t658) +
         ct_idx_332_tmp * t583 * t659 * 2.0) +
        ct_idx_44_tmp * t590 * t659 * 2.0) +
       ct_idx_315_tmp * t595 * t659) -
      t1124_tmp * ct_idx_270;
  t1286 =
      ((((((-(ct[25] * ct[127] * t658 * 2.0) + ct[27] * ct[284] * t658) -
           ct[26] * t557 * t658 * 2.0) +
          ((((((((ct[60] + ct[325]) + ct[64]) + ct[115]) + ct[169]) + ct[179]) +
             ct[197]) +
            ct[240]) +
           ct[250]) *
              t658) +
         ct_idx_325_tmp * t584 * t659 * 2.0) +
        ct_idx_40_tmp * t591 * t659 * 2.0) +
       ct_idx_316_tmp * t597 * t659) -
      t1120_tmp * ct_idx_272;
  t1287 =
      ((((((ct[27] * ct[201] * t658 * 2.0 + ct[25] * ct[283] * t658) -
           ct[26] * t554 * t658 * 2.0) +
          ((((((((ct[51] + ct[52]) + ct[58]) + ct[107]) + ct[160]) + ct[174]) +
             ct[200]) +
            ct[228]) +
           ct[243]) *
              t658) +
         ct_idx_40_tmp * t583 * t659 * 2.0) +
        ct_idx_316_tmp * t590 * t659 * 2.0) +
       ct_idx_325_tmp * t595 * t659) -
      t1120_tmp * ct_idx_270;
  t1288 =
      ((((((ct[26] * ct[201] * t658 * 2.0 + ct[27] * ct[282] * t658) -
           ct[25] * t558 * t658 * 2.0) +
          ((((((((ct[49] + ct[56]) + ct[61]) + ct[112]) + ct[165]) + ct[182]) +
             ct[199]) +
            ct[235]) +
           ct[246]) *
              t658) +
         ct_idx_36_tmp * t584 * t659 * 2.0) +
        ct_idx_309_tmp * t591 * t659 * 2.0) +
       ct_idx_326_tmp * t597 * t659) -
      t1116_tmp * ct_idx_272;
  t1289 =
      ((((((ct[25] * ct[201] * t658 * 2.0 + ct[26] * ct[286] * t658) -
           ct[27] * t555 * t658 * 2.0) +
          ((((((((ct[48] + ct[59]) + ct[65]) + ct[113]) + ct[168]) + ct[178]) +
             ct[197]) +
            ct[238]) +
           ct[251]) *
              t658) +
         ct_idx_44_tmp * t585 * t659 * 2.0) +
        ct_idx_315_tmp * t589 * t659 * 2.0) +
       ct_idx_332_tmp * t596 * t659) -
      t1124_tmp * ct_idx_271;
  t1354 = ((((ct[264] * t558 * t658 * 2.0 - ct[274] * t556 * t658 * 2.0) -
             ct[289] * ct[329] * t658) -
            ct_idx_337_tmp * t584 * t659 * 2.0) +
           t1349 * t589 * t659 * 2.0) +
          t1359 * t595 * t659;
  t1341 = (ct[21] * t587 + ct[20] * t592) + -ct[19] * t1354;
  t1353 = ((((ct[265] * t554 * t658 * 2.0 - ct[272] * t557 * t658 * 2.0) -
             ct[288] * ct[330] * t658) -
            t1357 * t583 * t659 * 2.0) +
           t1346 * t591 * t659 * 2.0) +
          t1352 * t596 * t659;
  t1342 = (ct[19] * t1178 + ct[21] * t1216) + -ct[20] * t1353;
  t1355 = ((((ct[263] * t555 * t658 * 2.0 - ct[273] * t553 * t658 * 2.0) -
             ct[287] * ct[331] * t658) -
            t1351 * t585 * t659 * 2.0) +
           t1345 * t590 * t659 * 2.0) +
          t1350 * t597 * t659;
  t1343 = (ct[20] * t586 + ct[19] * t594) + -ct[21] * t1355;
  t1344 = (ct[30] * t587 + ct[29] * t592) + -ct[28] * t1354;
  t1345 = (ct[28] * t1178 + ct[30] * t1216) + -ct[29] * t1353;
  t1346 = (ct[29] * t586 + ct[28] * t594) + -ct[30] * t1355;
  t1334 = ((((ct[263] * ct[281] * t658 - ct[201] * ct[287] * t658 * 2.0) +
             ct[273] * t556 * t658 * 2.0) -
            t1348 * t583 * t659 * 2.0) -
           ct_idx_28_tmp * t591 * t659 * 2.0) +
          t1347 * t596 * t659;
  t1340 = ((((ct[263] * t556 * t658 * 2.0 - ct[287] * t558 * t658 * 2.0) -
             ct[273] * ct[329] * t658) -
            t1347 * t589 * t659 * 2.0) +
           ct_idx_28_tmp * t584 * t659 * 2.0) +
          t1348 * t595 * t659;
  t1347 = (ct[21] * t588 + -ct[20] * t1334) + ct[19] * t1340;
  t1339 = ((((ct[264] * ct[284] * t658 - ct[201] * ct[289] * t658 * 2.0) +
             ct[274] * t557 * t658 * 2.0) -
            ct_idx_351_tmp * t585 * t659 * 2.0) -
           ct_idx_29_tmp * t590 * t659 * 2.0) +
          ct_idx_360_tmp * t597 * t659;
  t1353 = ((((ct[264] * t557 * t658 * 2.0 - ct[289] * t554 * t658 * 2.0) -
             ct[274] * ct[330] * t658) -
            ct_idx_360_tmp * t591 * t659 * 2.0) +
           ct_idx_29_tmp * t583 * t659 * 2.0) +
          ct_idx_351_tmp * t596 * t659;
  t1348 = (ct[19] * t593 + -ct[21] * t1339) + ct[20] * t1353;
  t1354 = ((((ct[265] * ct[285] * t658 - ct[201] * ct[288] * t658 * 2.0) +
             ct[272] * t553 * t658 * 2.0) -
            ct_idx_352_tmp * t584 * t659 * 2.0) -
           ct_idx_35_tmp * t589 * t659 * 2.0) +
          ct_idx_361_tmp * t595 * t659;
  t1355 = ((((ct[265] * t553 * t658 * 2.0 - ct[288] * t555 * t658 * 2.0) -
             ct[272] * ct[331] * t658) -
            ct_idx_361_tmp * t590 * t659 * 2.0) +
           ct_idx_35_tmp * t585 * t659 * 2.0) +
          ct_idx_352_tmp * t597 * t659;
  t1349 = (ct[20] * t1186 + -ct[19] * t1354) + ct[21] * t1355;
  t1350 = (ct[30] * t588 + -ct[29] * t1334) + ct[28] * t1340;
  t1351 = (ct[28] * t593 + -ct[30] * t1339) + ct[29] * t1353;
  t1352 = (ct[29] * t1186 + -ct[28] * t1354) + ct[30] * t1355;
  t1186 =
      ((((((ct[26] * t556 * t658 * 2.0 + ct[27] * t558 * t658 * 2.0) +
           ct[25] * ct[329] * t658) -
          ct_idx_309_tmp * t583 * t659 * 2.0) -
         ct_idx_326_tmp * t590 * t659 * 2.0) -
        ct_idx_36_tmp * t595 * t659) -
       t658 * ((((((((((((((ct[55] + ct[63]) + ct[75]) + ct[111]) + ct[116]) +
                        ct[143]) +
                       ct[164]) +
                      ct[177]) +
                     ct[183]) +
                    ct[188]) +
                   ct[224]) +
                  ct[226]) +
                 ct[230]) +
                ct[245]) +
               ct[249])) +
      t1116_tmp * ct_idx_270;
  t1357 = (ct[272] * t1288 + ct[288] * t1284) + ct[265] * t1186;
  ct_idx_337_tmp =
      ((((((ct[25] * t554 * t658 * 2.0 + ct[27] * t557 * t658 * 2.0) +
           ct[26] * ct[330] * t658) -
          ct_idx_316_tmp * t585 * t659 * 2.0) -
         ct_idx_325_tmp * t589 * t659 * 2.0) -
        ct_idx_40_tmp * t596 * t659) -
       t658 * ((((((((((((((ct[50] + ct[63]) + ct[76]) + ct[106]) + ct[116]) +
                        ct[144]) +
                       ct[158]) +
                      ct[167]) +
                     ct[177]) +
                    ct[184]) +
                   ct[227]) +
                  ct[231]) +
                 ct[236]) +
                ct[239]) +
               ct[252])) +
      t1120_tmp * ct_idx_271;
  t1359 = (ct[273] * t1287 + ct[287] * t1286) + ct[263] * ct_idx_337_tmp;
  t1178 =
      ((((((ct[25] * t553 * t658 * 2.0 + ct[26] * t555 * t658 * 2.0) +
           ct[27] * ct[331] * t658) -
          ct_idx_315_tmp * t584 * t659 * 2.0) -
         ct_idx_332_tmp * t591 * t659 * 2.0) -
        ct_idx_44_tmp * t597 * t659) -
       t658 * ((((((((((((((ct[50] + ct[55]) + ct[77]) + ct[106]) + ct[111]) +
                        ct[145]) +
                       ct[158]) +
                      ct[162]) +
                     ct[164]) +
                    ct[170]) +
                   ct[234]) +
                  ct[237]) +
                 ct[242]) +
                ct[244]) +
               ct[248])) +
      t1124_tmp * ct_idx_272;
  t1216 = (ct[274] * t1289 + ct[289] * t1285) + ct[264] * t1178;
  t1329 = (ct[21] * t1145 + ct[20] * t1148) - ct[19] * t1146;
  t1330 = (ct[20] * t1144 + ct[19] * t1147) - ct[21] * t1150;
  t1331 = (ct[19] * t1143 + ct[21] * t1151) - ct[20] * t1149;
  t593 = (ct[30] * t1145 + ct[29] * t1148) - ct[28] * t1146;
  t592 = (ct[29] * t1144 + ct[28] * t1147) - ct[30] * t1150;
  t1334 = (ct[28] * t1143 + ct[30] * t1151) - ct[29] * t1149;
  t586 = (ct[20] * t1176 + ct[21] * t1181) - ct[19] * t1231;
  t594 = (ct[19] * t1177 + ct[21] * t1184) - ct[20] * t1233;
  t588 = (ct[19] * t1183 + ct[20] * t1187) - ct[21] * t1238;
  t587 = (ct[29] * t1176 + ct[30] * t1181) - ct[28] * t1231;
  t1339 = (ct[28] * t1177 + ct[30] * t1184) - ct[29] * t1233;
  t1340 = (ct[28] * t1183 + ct[29] * t1187) - ct[30] * t1238;
  t1353 = (ct[273] * t1269 - ct[263] * t1270) + ct[287] * t1271;
  t1354 = (ct[272] * t1271 - ct[265] * t1269) + ct[288] * t1270;
  t1355 = (ct[274] * t1270 - ct[264] * t1271) + ct[289] * t1269;
  b_jacobian_h_to_b2Tb1[0] = 0.0;
  b_jacobian_h_to_b2Tb1[1] = 0.0;
  b_jacobian_h_to_b2Tb1[2] = 0.0;
  b_jacobian_h_to_b2Tb1[3] = 0.0;
  b_jacobian_h_to_b2Tb1[4] = 0.0;
  b_jacobian_h_to_b2Tb1[5] = 0.0;
  b_jacobian_h_to_b2Tb1[6] =
      (ct_idx_82 * t1134 - ct_idx_101 * t1135) - ct_idx_103 * t1136;
  b_jacobian_h_to_b2Tb1[7] =
      (ct_idx_83 * t1136 - ct_idx_102 * t1134) - ct_idx_104 * t1135;
  b_jacobian_h_to_b2Tb1[8] =
      (ct_idx_84 * t1135 - ct_idx_105 * t1134) - ct_idx_106 * t1136;
  std::memset(&b_jacobian_h_to_b2Tb1[9], 0, 9U * sizeof(double));
  b_jacobian_h_to_b2Tb1[18] =
      (ct_idx_82 * t1137 - ct_idx_101 * t1138) - ct_idx_103 * t1139;
  b_jacobian_h_to_b2Tb1[19] =
      (ct_idx_83 * t1139 - ct_idx_102 * t1137) - ct_idx_104 * t1138;
  b_jacobian_h_to_b2Tb1[20] =
      (ct_idx_84 * t1138 - ct_idx_105 * t1137) - ct_idx_106 * t1139;
  std::memset(&b_jacobian_h_to_b2Tb1[21], 0, 9U * sizeof(double));
  b_jacobian_h_to_b2Tb1[30] =
      (ct_idx_82 * t1140 - ct_idx_101 * t1141) - ct_idx_103 * t1142;
  b_jacobian_h_to_b2Tb1[31] =
      (ct_idx_83 * t1142 - ct_idx_102 * t1140) - ct_idx_104 * t1141;
  b_jacobian_h_to_b2Tb1[32] =
      (ct_idx_84 * t1141 - ct_idx_105 * t1140) - ct_idx_106 * t1142;
  std::memset(&b_jacobian_h_to_b2Tb1[33], 0, 9U * sizeof(double));
  b_jacobian_h_to_b2Tb1[42] =
      ((((ct_idx_76 * t593 - ct_idx_89 * t592) - ct_idx_91 * t1334) +
        ct_idx_82 * t1353) -
       ct_idx_101 * t1354) -
      ct_idx_103 * t1355;
  b_jacobian_h_to_b2Tb1[43] =
      ((((ct_idx_77 * t1334 - ct_idx_90 * t593) - ct_idx_92 * t592) +
        ct_idx_83 * t1355) -
       ct_idx_102 * t1353) -
      ct_idx_104 * t1354;
  b_jacobian_h_to_b2Tb1[44] =
      ((((ct_idx_78 * t592 - ct_idx_93 * t593) - ct_idx_94 * t1334) +
        ct_idx_84 * t1354) -
       ct_idx_105 * t1353) -
      ct_idx_106 * t1355;
  b_jacobian_h_to_b2Tb1[45] =
      ((-ct[9] * (((t629 - t636) + t642) - t639) + ct_idx_79 * t1329) -
       ct_idx_95 * t1330) -
      ct_idx_97 * t1331;
  t1354 = t630 + t635;
  b_jacobian_h_to_b2Tb1[46] =
      ((-ct[10] * ((t1354 - t640) - t641) + ct_idx_80 * t1331) -
       ct_idx_96 * t1329) -
      ct_idx_98 * t1330;
  b_jacobian_h_to_b2Tb1[47] =
      ((-ct[11] * (((t631 + t637) - t644) - t634) + ct_idx_81 * t1330) -
       ct_idx_99 * t1329) -
      ct_idx_100 * t1331;
  b_jacobian_h_to_b2Tb1[48] = 0.0;
  b_jacobian_h_to_b2Tb1[49] = 0.0;
  b_jacobian_h_to_b2Tb1[50] = 0.0;
  b_jacobian_h_to_b2Tb1[51] = 0.0;
  b_jacobian_h_to_b2Tb1[52] = 0.0;
  b_jacobian_h_to_b2Tb1[53] = 0.0;
  t1353 = (-(ct[274] * t1284) + ct[264] * t1288) + ct[289] * t1186;
  t1355 = (ct[263] * t1284 - ct[287] * t1288) + ct[273] * t1186;
  b_jacobian_h_to_b2Tb1[54] =
      ((((ct_idx_76 * t1350 - ct_idx_89 * t587) + ct_idx_91 * t1344) -
        ct_idx_101 * t1357) -
       ct_idx_82 * t1355) +
      ct_idx_103 * t1353;
  b_jacobian_h_to_b2Tb1[55] =
      ((((-ct_idx_77 * t1344 - ct_idx_92 * t587) - ct_idx_90 * t1350) -
        ct_idx_104 * t1357) -
       ct_idx_83 * t1353) +
      ct_idx_102 * t1355;
  b_jacobian_h_to_b2Tb1[56] =
      ((((ct_idx_78 * t587 + ct_idx_94 * t1344) + ct_idx_84 * t1357) -
        ct_idx_93 * t1350) +
       ct_idx_105 * t1355) +
      ct_idx_106 * t1353;
  b_jacobian_h_to_b2Tb1[57] =
      ((ct_idx_79 * t1347 - ct_idx_95 * t586) + ct_idx_97 * t1341) -
      ct[9] * (((t632 + t633) - t638) - t643);
  b_jacobian_h_to_b2Tb1[58] =
      ((ct[10] * (((t631 - t637) + t644) - t634) - ct_idx_80 * t1341) -
       ct_idx_98 * t586) -
      ct_idx_96 * t1347;
  b_jacobian_h_to_b2Tb1[59] =
      ((ct_idx_81 * t586 + ct_idx_100 * t1341) - ct_idx_99 * t1347) -
      ct[11] * ((t1354 + t640) + t641);
  b_jacobian_h_to_b2Tb1[60] = 0.0;
  b_jacobian_h_to_b2Tb1[61] = 0.0;
  b_jacobian_h_to_b2Tb1[62] = 0.0;
  b_jacobian_h_to_b2Tb1[63] = 0.0;
  b_jacobian_h_to_b2Tb1[64] = 0.0;
  b_jacobian_h_to_b2Tb1[65] = 0.0;
  t1354 = (ct[264] * t1286 - ct[289] * t1287) + ct[274] * ct_idx_337_tmp;
  t1353 = (ct[265] * t1287 - ct[272] * t1286) + ct[288] * ct_idx_337_tmp;
  b_jacobian_h_to_b2Tb1[66] =
      ((((ct_idx_76 * t1339 + ct_idx_89 * t1345) + ct_idx_82 * t1359) -
        ct_idx_91 * t1351) +
       ct_idx_101 * t1353) +
      ct_idx_103 * t1354;
  b_jacobian_h_to_b2Tb1[67] =
      ((((ct_idx_77 * t1351 - ct_idx_90 * t1339) + ct_idx_92 * t1345) -
        ct_idx_102 * t1359) -
       ct_idx_83 * t1354) +
      ct_idx_104 * t1353;
  b_jacobian_h_to_b2Tb1[68] =
      ((((-ct_idx_78 * t1345 - ct_idx_93 * t1339) - ct_idx_94 * t1351) -
        ct_idx_105 * t1359) -
       ct_idx_84 * t1353) +
      ct_idx_106 * t1354;
  b_jacobian_h_to_b2Tb1[69] =
      ((ct_idx_79 * t594 + ct_idx_95 * t1342) - ct_idx_97 * t1348) -
      ct[9] * (((t631 + t634) + t637) + t644);
  b_jacobian_h_to_b2Tb1[70] =
      ((ct_idx_80 * t1348 - ct_idx_96 * t594) + ct_idx_98 * t1342) -
      ct[10] * (((t632 + t638) - t633) - t643);
  t1354 = t629 + t636;
  b_jacobian_h_to_b2Tb1[71] =
      ((ct[11] * ((t1354 - t642) - t639) - ct_idx_81 * t1342) -
       ct_idx_99 * t594) -
      ct_idx_100 * t1348;
  b_jacobian_h_to_b2Tb1[72] = 0.0;
  b_jacobian_h_to_b2Tb1[73] = 0.0;
  b_jacobian_h_to_b2Tb1[74] = 0.0;
  b_jacobian_h_to_b2Tb1[75] = 0.0;
  b_jacobian_h_to_b2Tb1[76] = 0.0;
  b_jacobian_h_to_b2Tb1[77] = 0.0;
  t1353 = (ct[265] * t1285 - ct[288] * t1289) + ct[272] * t1178;
  t1355 = (-(ct[273] * t1285) + ct[263] * t1289) + ct[287] * t1178;
  b_jacobian_h_to_b2Tb1[78] =
      ((((-ct_idx_76 * t1346 - ct_idx_91 * t1340) - ct_idx_89 * t1352) -
        ct_idx_103 * t1216) -
       ct_idx_82 * t1355) +
      ct_idx_101 * t1353;
  b_jacobian_h_to_b2Tb1[79] =
      ((((ct_idx_77 * t1340 + ct_idx_90 * t1346) - ct_idx_92 * t1352) +
        ct_idx_83 * t1216) +
       ct_idx_104 * t1353) +
      ct_idx_102 * t1355;
  b_jacobian_h_to_b2Tb1[80] =
      ((((ct_idx_78 * t1352 - ct_idx_94 * t1340) + ct_idx_93 * t1346) -
        ct_idx_106 * t1216) -
       ct_idx_84 * t1353) +
      ct_idx_105 * t1355;
  b_jacobian_h_to_b2Tb1[81] =
      ((ct[9] * (((t630 - t635) + t640) - t641) - ct_idx_79 * t1343) -
       ct_idx_97 * t588) -
      ct_idx_95 * t1349;
  b_jacobian_h_to_b2Tb1[82] =
      ((ct_idx_80 * t588 + ct_idx_96 * t1343) - ct_idx_98 * t1349) -
      ct[10] * ((t1354 + t639) + t642);
  b_jacobian_h_to_b2Tb1[83] =
      ((ct_idx_81 * t1349 - ct_idx_100 * t588) + ct_idx_99 * t1343) -
      ct[11] * (((t632 + t643) - t633) - t638);
}

//
// Arguments    : double u0
//                double u1
// Return Type  : double
//
static double rt_powd_snf(double u0, double u1)
{
  double y;
  if (std::isnan(u0) || std::isnan(u1)) {
    y = rtNaN;
  } else {
    double d;
    double d1;
    d = std::abs(u0);
    d1 = std::abs(u1);
    if (std::isinf(u1)) {
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
      y = std::pow(u0, u1);
    }
  }
  return y;
}

//
// jacobian_h_to_b2Tb1
//     jacobian_h_to_b2Tb1 =
//     jacobian_h_to_b2Tb1(IN1,IN2,IN3,IN4,IN5,IN6,IN7,IN8,IN9,IN10,IN11,IN12,IN13,IN14,IN15,IN16,IN17,IN18,IN19,IN20,IN21,IN22,IN23,IN24)
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
//                double b_jacobian_h_to_b2Tb1[84]
// Return Type  : void
//
void jacobian_h_to_b2Tb1(const double[3], const double in2[4], const double[3],
                         const double[3], const double[3], const double[4],
                         const double in7[3], const double in8[4],
                         const double in9[3], const double in10[4],
                         const double[3], const double[3], const double in13[3],
                         const double in14[3], const double[3],
                         const double in16[4], const double[3], const double[4],
                         const double[3], const double in20[4], const double[6],
                         const double[6], const double in23[6],
                         const double in24[6], double b_jacobian_h_to_b2Tb1[84])
{
  double b_in24[343];
  double b_in24_tmp;
  double c_in24_tmp;
  double d_in24_tmp;
  double e_in24_tmp;
  double f_in24_tmp;
  double g_in24_tmp;
  double h_in24_tmp;
  double i_in24_tmp;
  double in24_tmp;
  double j_in24_tmp;
  double t100;
  double t102;
  double t103;
  double t104;
  double t105;
  double t106;
  double t107;
  double t108;
  double t109;
  double t110;
  double t111;
  double t142;
  double t143;
  double t144;
  double t151;
  double t152;
  double t153;
  double t182;
  double t184;
  double t185;
  double t186;
  double t187;
  double t188;
  double t2;
  double t205;
  double t207;
  double t210;
  double t212;
  double t214;
  double t24;
  double t25;
  double t257;
  double t258;
  double t259;
  double t26;
  double t260;
  double t261;
  double t262;
  double t263;
  double t264;
  double t265;
  double t266;
  double t267;
  double t268;
  double t28;
  double t289;
  double t29;
  double t293;
  double t295;
  double t297;
  double t298;
  double t3;
  double t30;
  double t302;
  double t307;
  double t310;
  double t32;
  double t323;
  double t324;
  double t328;
  double t33;
  double t35;
  double t363;
  double t364;
  double t365;
  double t366;
  double t366_tmp;
  double t367;
  double t367_tmp;
  double t368;
  double t368_tmp;
  double t4;
  double t411;
  double t412;
  double t413;
  double t414;
  double t415;
  double t416;
  double t444;
  double t445;
  double t446;
  double t447;
  double t46;
  double t47;
  double t48;
  double t49;
  double t5;
  double t50;
  double t51;
  double t52;
  double t52_tmp;
  double t53;
  double t53_tmp;
  double t55;
  double t55_tmp;
  double t57;
  double t57_tmp;
  double t58;
  double t58_tmp;
  double t6;
  double t61;
  double t62;
  double t64;
  double t64_tmp;
  double t65;
  double t66;
  double t68;
  double t69;
  double t7;
  double t70;
  double t71;
  double t72;
  double t73;
  double t74;
  double t75;
  double t76;
  double t77;
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
  double t91;
  double t92;
  double t93;
  double t97;
  //     This function was generated by the Symbolic Math Toolbox version 23.2.
  //     31-Mar-2025 10:49:21
  //  input
  //  {bpo; bQo; ovo; oomegao; b1pe1_b1; b1Qe1; b2pe2_b2; b2Qe2; b2pb1; b2Qb1;
  //  ...
  //      b1pe1_dot; b1_omega_e1; b2pe2_dot; b2_omega_e2 ;...
  //      bpb1; bQb1; opg1;oQg1; opg2;oQg2;K_1_diag;B_1_diag;K_2_diag;B_2_diag})
  t2 = in10[0] * in10[1];
  t3 = in10[0] * in10[2];
  t4 = in10[0] * in10[3];
  t5 = in10[1] * in10[2];
  t6 = in10[1] * in10[3];
  t7 = in10[2] * in10[3];
  t24 = in10[1] * 4.0;
  t25 = in10[2] * 4.0;
  t26 = in10[3] * 4.0;
  t28 = in10[1] * in10[1];
  t29 = rt_powd_snf(in10[1], 3.0);
  t30 = in10[2] * in10[2];
  t32 = rt_powd_snf(in10[2], 3.0);
  t33 = in10[3] * in10[3];
  t35 = rt_powd_snf(in10[3], 3.0);
  t52_tmp = in10[0] * in9[0];
  t52 = t52_tmp * 2.0;
  t53_tmp = in10[0] * in9[1];
  t53 = t53_tmp * 2.0;
  t55_tmp = in10[0] * in9[2];
  t55 = t55_tmp * 2.0;
  t57_tmp = in9[0] * in10[2];
  t57 = t57_tmp * 2.0;
  t58_tmp = in10[1] * in9[2];
  t58 = t58_tmp * 2.0;
  t64_tmp = in9[1] * in10[3];
  t64 = t64_tmp * 2.0;
  t70 = in16[0] * in16[1] * 2.0;
  t71 = in16[0] * in16[2] * 2.0;
  t72 = in16[0] * in16[3] * 2.0;
  t73 = in16[1] * in16[2] * 2.0;
  t74 = in16[1] * in16[3] * 2.0;
  t75 = in16[2] * in16[3] * 2.0;
  t76 = in2[0] * in2[1] * 2.0;
  t77 = in2[0] * in2[2] * 2.0;
  t78 = in2[0] * in2[3] * 2.0;
  t79 = in2[1] * in2[2] * 2.0;
  t80 = in2[1] * in2[3] * 2.0;
  t81 = in2[2] * in2[3] * 2.0;
  t82 = in20[0] * in20[1] * 2.0;
  t83 = in20[0] * in20[2] * 2.0;
  t84 = in20[0] * in20[3] * 2.0;
  t85 = in20[1] * in20[2] * 2.0;
  t86 = in20[1] * in20[3] * 2.0;
  t87 = in20[2] * in20[3] * 2.0;
  t91 = in10[1] * 8.0;
  t92 = in10[2] * 8.0;
  t93 = in10[3] * 8.0;
  t46 = t2 * 2.0;
  t47 = t3 * 2.0;
  t48 = t4 * 2.0;
  t49 = t5 * 2.0;
  t50 = t6 * 2.0;
  t51 = t7 * 2.0;
  t61 = in9[1] * t24;
  t62 = in9[0] * t25;
  t65 = in9[2] * t24;
  t66 = in9[0] * t26;
  t68 = in9[2] * t25;
  t69 = in9[1] * t26;
  t97 = in10[1] * t24;
  t100 = in10[2] * t25;
  t102 = t24 * t29;
  t103 = in10[3] * t26;
  t104 = t25 * t32;
  t105 = t26 * t35;
  t106 = in16[1] * in16[1] * 2.0;
  t107 = in16[2] * in16[2] * 2.0;
  t108 = in16[3] * in16[3] * 2.0;
  t109 = in2[1] * in2[1] * 2.0;
  t110 = in2[2] * in2[2] * 2.0;
  t111 = in2[3] * in2[3] * 2.0;
  t142 = in9[0] * t91;
  t143 = in9[1] * t92;
  t144 = in9[2] * t93;
  t151 = in20[1] * in20[1] * 2.0;
  t152 = in20[2] * in20[2] * 2.0;
  t153 = in20[3] * in20[3] * 2.0;
  t182 = t28 * -4.0;
  t184 = t30 * -4.0;
  t185 = t33 * -4.0;
  t186 = t29 * 16.0;
  t187 = t32 * 16.0;
  t188 = t35 * 16.0;
  t257 = t2 * t91;
  t258 = in10[0] * t2 * 8.0;
  t259 = t3 * t92;
  t260 = in10[0] * t3 * 8.0;
  t261 = t4 * t93;
  t262 = t5 * t92;
  t263 = in10[0] * t4 * 8.0;
  t264 = t5 * t91;
  t265 = t6 * t93;
  t266 = t6 * t91;
  t267 = t7 * t93;
  t268 = t7 * t92;
  t289 = t2 * t3 * 4.0;
  t293 = t2 * t4 * 4.0;
  t295 = t3 * t4 * 4.0;
  t297 = t5 * t7 * 4.0;
  t298 = t5 * t6 * 4.0;
  t363 = t2 * t2 * 4.0;
  t364 = t3 * t3 * 4.0;
  t365 = t4 * t4 * 4.0;
  t366_tmp = t5 * t5;
  t366 = t366_tmp * 4.0;
  t367_tmp = t6 * t6;
  t367 = t367_tmp * 4.0;
  t368_tmp = t7 * t7;
  t368 = t368_tmp * 4.0;
  t411 = t76 + t81;
  t412 = t77 + t80;
  t413 = t78 + t79;
  t414 = t82 + t87;
  t415 = t83 + t86;
  t416 = t84 + t85;
  t205 = t5 * t100;
  t207 = t5 * t97;
  t210 = t6 * t103;
  t212 = t6 * t97;
  t214 = t7 * t103;
  t97 = t7 * t100;
  t302 = t5 * t62;
  t307 = t6 * t66;
  t310 = t5 * t61;
  t323 = t7 * t69;
  t324 = t6 * t65;
  t328 = t7 * t68;
  t444 = ((in2[0] * in20[1] + in20[0] * in2[1]) + in2[2] * in20[3]) -
         in20[2] * in2[3];
  t445 = ((in2[0] * in20[2] + in20[0] * in2[2]) + in20[1] * in2[3]) -
         in2[1] * in20[3];
  t446 = ((in2[0] * in20[3] + in2[1] * in20[2]) + in20[0] * in2[3]) -
         in20[1] * in2[2];
  t447 = ((in2[1] * in20[1] + in2[2] * in20[2]) + in2[3] * in20[3]) -
         in2[0] * in20[0];
  b_in24[0] = in24[0];
  b_in24[1] = in24[1];
  b_in24[2] = in24[2];
  b_in24[3] = in24[3];
  b_in24[4] = in24[4];
  b_in24[5] = in24[5];
  b_in24[6] = in23[0];
  b_in24[7] = in23[1];
  b_in24[8] = in23[2];
  b_in24[9] = in23[3];
  b_in24[10] = in23[4];
  b_in24[11] = in23[5];
  b_in24[12] = in10[1];
  b_in24[13] = in10[2];
  b_in24[14] = in10[3];
  b_in24[15] = in8[0];
  b_in24[16] = in8[1];
  b_in24[17] = in8[2];
  b_in24[18] = in8[3];
  b_in24[19] = in14[0];
  b_in24[20] = in14[1];
  b_in24[21] = in14[2];
  b_in24[22] = in9[0];
  b_in24[23] = in9[1];
  b_in24[24] = in9[2];
  b_in24[25] = in7[0];
  b_in24[26] = in7[1];
  b_in24[27] = in7[2];
  b_in24[28] = in13[0];
  b_in24[29] = in13[1];
  b_in24[30] = in13[2];
  b_in24[31] = in16[3];
  b_in24[32] = t35 * 2.0;
  b_in24[33] = t102;
  b_in24[34] = t104;
  b_in24[35] = t105;
  b_in24[36] = -t2;
  b_in24[37] = -t46;
  b_in24[38] = -t3;
  b_in24[39] = -t47;
  b_in24[40] = -t4;
  b_in24[41] = -t5;
  b_in24[42] = -t48;
  b_in24[43] = -t49;
  b_in24[44] = -t6;
  b_in24[45] = -t50;
  b_in24[46] = -t7;
  b_in24[47] = -t51;
  b_in24[48] = -t52;
  b_in24[49] = -t53;
  in24_tmp = in9[0] * in10[1];
  b_in24[50] = -(in24_tmp * 2.0);
  b_in24[51] = -t55;
  b_in24_tmp = in10[1] * in9[1];
  b_in24[52] = -(b_in24_tmp * 2.0);
  b_in24[53] = -t57;
  b_in24[54] = -t58;
  c_in24_tmp = in9[1] * in10[2];
  b_in24[55] = -(c_in24_tmp * 2.0);
  d_in24_tmp = in9[0] * in10[3];
  b_in24[56] = -(d_in24_tmp * 2.0);
  b_in24[57] = b_in24_tmp * -4.0;
  b_in24[58] = t57_tmp * -4.0;
  e_in24_tmp = in10[2] * in9[2];
  b_in24[59] = -(e_in24_tmp * 2.0);
  b_in24[60] = -t64;
  b_in24[61] = t58_tmp * -4.0;
  b_in24[62] = d_in24_tmp * -4.0;
  f_in24_tmp = in9[2] * in10[3];
  b_in24[63] = -(f_in24_tmp * 2.0);
  b_in24[64] = e_in24_tmp * -4.0;
  b_in24[65] = t64_tmp * -4.0;
  b_in24[66] = in9[0] * t47;
  b_in24[67] = in9[2] * t46;
  b_in24[68] = in9[1] * t48;
  b_in24[69] = -(t28 * 2.0);
  b_in24[70] = -(t30 * 2.0);
  b_in24[71] = t182;
  b_in24[72] = -(t33 * 2.0);
  b_in24[73] = t184;
  b_in24[74] = t185;
  b_in24[75] = in24_tmp * -8.0;
  b_in24[76] = c_in24_tmp * -8.0;
  b_in24[77] = f_in24_tmp * -8.0;
  b_in24[78] = in10[0] * t46;
  b_in24[79] = in10[0] * t47;
  b_in24[80] = in10[2] * t49;
  b_in24[81] = in10[0] * t48;
  b_in24[82] = in10[1] * t49;
  b_in24[83] = t30 * t49;
  b_in24[84] = in10[3] * t50;
  b_in24[85] = in10[1] * t50;
  b_in24[86] = t2;
  b_in24[87] = t28 * t49;
  b_in24[88] = t33 * t50;
  b_in24[89] = in10[3] * t51;
  b_in24[90] = t28 * t50;
  b_in24[91] = in10[2] * t51;
  b_in24[92] = t205;
  b_in24[93] = t33 * t51;
  b_in24[94] = t207;
  b_in24[95] = t30 * t51;
  t100 = in10[2] * t5;
  b_in24[96] = t100 * 6.0;
  b_in24[97] = t210;
  t35 = in10[1] * t5;
  b_in24[98] = t35 * 6.0;
  b_in24[99] = t212;
  t50 = in10[3] * t6;
  b_in24[100] = t50 * 6.0;
  b_in24[101] = t214;
  t51 = in10[1] * t6;
  b_in24[102] = t51 * 6.0;
  b_in24[103] = t97;
  g_in24_tmp = in10[3] * t7;
  b_in24[104] = g_in24_tmp * 6.0;
  h_in24_tmp = in10[2] * t7;
  b_in24[105] = h_in24_tmp * 6.0;
  b_in24[106] = in9[0] * t24 * t28;
  b_in24[107] = t28 * t61;
  b_in24[108] = in9[0] * t102;
  b_in24[109] = t30 * t62;
  b_in24[110] = t28 * t65;
  b_in24[111] = in9[1] * t25 * t30;
  b_in24[112] = t33 * t66;
  b_in24[113] = t30 * t68;
  b_in24[114] = in9[1] * t104;
  b_in24[115] = t33 * t69;
  b_in24[116] = in9[2] * t26 * t33;
  b_in24[117] = in9[2] * t105;
  b_in24[118] = in9[1] * t2 * -2.0;
  b_in24[119] = in9[0] * t4 * -2.0;
  i_in24_tmp = in9[0] * t5;
  b_in24[120] = i_in24_tmp * -2.0;
  b_in24[121] = in9[2] * t3 * -2.0;
  j_in24_tmp = in9[1] * t5;
  b_in24[122] = j_in24_tmp * -2.0;
  b_in24[123] = in9[0] * t6 * -2.0;
  b_in24[124] = in9[2] * t6 * -2.0;
  b_in24[125] = in9[1] * t7 * -2.0;
  b_in24[126] = in9[2] * t7 * -2.0;
  b_in24[127] = in10[0] + t5 * t26;
  b_in24[128] = in10[1] + t3 * t26;
  b_in24[129] = in10[2] + t2 * t26;
  b_in24[130] = in10[3] + t2 * t25;
  b_in24[131] = t257;
  b_in24[132] = t259;
  b_in24[133] = t261;
  b_in24[134] = in9[1] * t28 * -2.0;
  b_in24[135] = in9[0] * t30 * -2.0;
  b_in24[136] = in9[0] * t182;
  b_in24[137] = in9[2] * t28 * -2.0;
  b_in24[138] = in9[0] * t33 * -2.0;
  b_in24[139] = in9[2] * t30 * -2.0;
  b_in24[140] = in9[1] * t33 * -2.0;
  b_in24[141] = in9[1] * t184;
  b_in24[142] = in9[2] * t185;
  b_in24[143] = in9[0] * t186;
  b_in24[144] = in9[1] * t187;
  b_in24[145] = in9[2] * t188;
  b_in24[146] = t3 * t46;
  b_in24[147] = t4 * t46;
  b_in24[148] = t289;
  b_in24[149] = t4 * t47;
  b_in24[150] = t33 * t49;
  b_in24[151] = t7 * t49;
  b_in24[152] = t293;
  b_in24[153] = t6 * t49;
  b_in24[154] = t295;
  b_in24[155] = t5 * t103;
  b_in24[156] = t297;
  b_in24[157] = t298;
  b_in24[158] = t52_tmp * t2 * 4.0;
  b_in24[159] = t3;
  b_in24[160] = t53_tmp * t2 * 4.0;
  b_in24[161] = t52_tmp * t3 * 4.0;
  b_in24[162] = t302;
  b_in24[163] = t55_tmp * t2 * 4.0;
  b_in24[164] = t53_tmp * t3 * 4.0;
  b_in24[165] = t52_tmp * t4 * 4.0;
  b_in24[166] = in10[2] * t302;
  b_in24[167] = t307;
  b_in24[168] = t55_tmp * t3 * 4.0;
  b_in24[169] = t53_tmp * t4 * 4.0;
  b_in24[170] = t310;
  b_in24[171] = in9[0] * t207;
  b_in24[172] = t5 * t68;
  b_in24[173] = in9[1] * t205;
  b_in24[174] = t6 * t69;
  b_in24[175] = in10[3] * t307;
  b_in24[176] = t7 * t66;
  b_in24[177] = t55_tmp * t4 * 4.0;
  b_in24[178] = t5 * t65;
  b_in24[179] = t6 * t61;
  b_in24[180] = in10[1] * t310;
  b_in24[181] = in9[0] * t212;
  b_in24[182] = t7 * t62;
  b_in24[183] = t323;
  b_in24[184] = t324;
  b_in24[185] = in9[2] * t210;
  b_in24[186] = in10[3] * t323;
  b_in24[187] = in10[1] * t324;
  b_in24[188] = t328;
  b_in24[189] = in9[1] * t97;
  b_in24[190] = in9[2] * t214;
  b_in24[191] = in10[2] * t328;
  t97 = in9[0] * t2;
  b_in24[192] = t97 * t92;
  b_in24[193] = t2 * t143;
  b_in24[194] = t97 * t93;
  b_in24[195] = t2 * t144;
  b_in24[196] = in9[1] * t3 * t93;
  b_in24[197] = i_in24_tmp * t93;
  b_in24[198] = t3 * t144;
  b_in24[199] = j_in24_tmp * t93;
  b_in24[200] = t5 * t144;
  i_in24_tmp = in10[3] * t5;
  b_in24[201] = in10[0] + i_in24_tmp * -4.0;
  b_in24[202] = in10[1] + in10[3] * t3 * -4.0;
  b_in24[203] = in10[2] + in10[3] * t2 * -4.0;
  b_in24[204] = in10[3] + in10[2] * t2 * -4.0;
  b_in24[205] = in9[0] * t289;
  b_in24[206] = in9[1] * t289;
  b_in24[207] = in9[0] * t293;
  b_in24[208] = i_in24_tmp * t66;
  b_in24[209] = in9[0] * t297;
  b_in24[210] = in9[2] * t293;
  b_in24[211] = in9[1] * t295;
  b_in24[212] = i_in24_tmp * t69;
  b_in24[213] = in9[2] * t295;
  b_in24[214] = in9[1] * t298;
  b_in24[215] = in9[2] * t297;
  b_in24[216] = in9[2] * t298;
  b_in24[217] = t363;
  b_in24[218] = t364;
  b_in24[219] = t365;
  b_in24[220] = t366;
  b_in24[221] = t367;
  b_in24[222] = t368;
  b_in24[223] = t2 * t142;
  b_in24[224] = in9[0] * t258;
  b_in24[225] = t3 * t143;
  b_in24[226] = in9[0] * t262;
  b_in24[227] = in9[1] * t260;
  b_in24[228] = t5 * t142;
  b_in24[229] = t5 * t143;
  b_in24[230] = in9[0] * t265;
  b_in24[231] = in9[1] * t264;
  b_in24[232] = t6 * t142;
  b_in24[233] = t4 * t144;
  b_in24[234] = in9[2] * t263;
  b_in24[235] = t6 * t144;
  b_in24[236] = in9[1] * t267;
  b_in24[237] = in9[2] * t266;
  b_in24[238] = t7 * t143;
  b_in24[239] = t57_tmp * t5 * 12.0;
  b_in24[240] = t7 * t144;
  b_in24[241] = in24_tmp * t5 * 12.0;
  b_in24[242] = in9[2] * t268;
  b_in24[243] = c_in24_tmp * t5 * 12.0;
  b_in24[244] = d_in24_tmp * t6 * 12.0;
  b_in24[245] = b_in24_tmp * t5 * 12.0;
  b_in24[246] = in24_tmp * t6 * 12.0;
  b_in24[247] = f_in24_tmp * t6 * 12.0;
  b_in24[248] = t64_tmp * t7 * 12.0;
  b_in24[249] = t58_tmp * t6 * 12.0;
  b_in24[250] = c_in24_tmp * t7 * 12.0;
  b_in24[251] = f_in24_tmp * t7 * 12.0;
  b_in24[252] = e_in24_tmp * t7 * 12.0;
  b_in24[253] = in9[0] * t363;
  b_in24[254] = t4;
  b_in24[255] = in9[1] * t364;
  b_in24[256] = in9[0] * t366;
  b_in24[257] = in9[1] * t366;
  b_in24[258] = in9[0] * t367;
  b_in24[259] = in9[2] * t365;
  b_in24[260] = in9[2] * t367;
  b_in24[261] = in9[1] * t368;
  b_in24[262] = in9[2] * t368;
  b_in24[263] = t70 + t75;
  b_in24[264] = t71 + t74;
  b_in24[265] = t72 + t73;
  b_in24[266] = t411;
  b_in24[267] = t412;
  b_in24[268] = t413;
  b_in24[269] = t414;
  b_in24[270] = t415;
  b_in24[271] = t416;
  b_in24[272] = t70 - t75;
  b_in24[273] = t71 - t74;
  b_in24[274] = t72 - t73;
  b_in24[275] = t76 - t81;
  b_in24[276] = t77 - t80;
  b_in24[277] = t78 - t79;
  b_in24[278] = t82 - t87;
  b_in24[279] = t83 - t86;
  b_in24[280] = t84 - t85;
  b_in24[281] = t24 + t100 * -8.0;
  b_in24[282] = t24 + t50 * -8.0;
  b_in24[283] = t25 + t35 * -8.0;
  b_in24[284] = t25 + g_in24_tmp * -8.0;
  b_in24[285] = t26 + t51 * -8.0;
  b_in24[286] = t26 + h_in24_tmp * -8.0;
  b_in24[287] = (t106 + t107) - 1.0;
  b_in24[288] = (t106 + t108) - 1.0;
  b_in24[289] = (t107 + t108) - 1.0;
  b_in24[290] = (t109 + t110) - 1.0;
  b_in24[291] = (t109 + t111) - 1.0;
  b_in24[292] = (t110 + t111) - 1.0;
  b_in24[293] = (t151 + t152) - 1.0;
  b_in24[294] = (t151 + t153) - 1.0;
  b_in24[295] = (t152 + t153) - 1.0;
  b_in24[296] = t447;
  b_in24[297] = in16[0] * t444;
  b_in24[298] = in16[0] * t445;
  b_in24[299] = in16[0] * t446;
  b_in24[300] = in16[0] * t447;
  b_in24[301] = in16[1] * t444;
  b_in24[302] = in16[1] * t445;
  b_in24[303] = in16[1] * t446;
  b_in24[304] = in16[1] * t447;
  b_in24[305] = in16[2] * t444;
  b_in24[306] = in16[2] * t445;
  b_in24[307] = in16[2] * t446;
  b_in24[308] = in16[2] * t447;
  b_in24[309] = t46;
  b_in24[310] = in16[3] * t444;
  b_in24[311] = in16[3] * t445;
  b_in24[312] = in16[3] * t446;
  b_in24[313] = in24[0] * t411 * t416;
  b_in24[314] = in24[1] * t412 * t414;
  b_in24[315] = t47;
  b_in24[316] = in24[2] * t413 * t415;
  b_in24[317] = in24[3] * t411 * t416;
  b_in24[318] = in24[4] * t412 * t414;
  b_in24[319] = in24[5] * t413 * t415;
  b_in24[320] = in23[0] * t411 * t416;
  b_in24[321] = in23[1] * t412 * t414;
  b_in24[322] = in23[2] * t413 * t415;
  b_in24[323] = t48;
  b_in24[324] = (t257 + t259) + t261;
  b_in24[325] = t52;
  b_in24[326] = t53;
  b_in24[327] = t55;
  b_in24[328] = t57;
  in24_tmp = (-t91 + t186) + t258;
  b_in24[329] = (in24_tmp + t262) + t265;
  b_in24_tmp = (-t92 + t187) + t260;
  b_in24[330] = (b_in24_tmp + t264) + t267;
  c_in24_tmp = (-t93 + t188) + t263;
  b_in24[331] = (c_in24_tmp + t266) + t268;
  b_in24[332] = t58;
  b_in24[333] = (in24_tmp + t100 * 16.0) + t50 * 16.0;
  b_in24[334] = (b_in24_tmp + t35 * 16.0) + g_in24_tmp * 16.0;
  b_in24[335] = (c_in24_tmp + t51 * 16.0) + h_in24_tmp * 16.0;
  b_in24[336] = t64;
  b_in24[337] =
      (((((((((((t102 + t104) + t105) + t182) + t184) + t185) + t363) + t364) +
          t365) +
         t366_tmp * 8.0) +
        t367_tmp * 8.0) +
       t368_tmp * 8.0) +
      1.0;
  b_in24[338] = -in10[1];
  b_in24[339] = -in10[2];
  b_in24[340] = -in10[3];
  b_in24[341] = t29 * 2.0;
  b_in24[342] = t32 * 2.0;
  ft_1(b_in24, b_jacobian_h_to_b2Tb1);
}

//
// File trailer for jacobian_h_to_b2Tb1.cpp
//
// [EOF]
//
