  t2 = Q1*p1*2.0;
  t3 = Q1*p2*2.0;
  t4 = Q2*p1*2.0;
  t5 = Q1*p3*2.0;
  t6 = Q2*p2*2.0;
  t7 = Q3*p1*2.0;
  t8 = Q2*p3*2.0;
  t9 = Q3*p2*2.0;
  t10 = Q4*p1*2.0;
  t11 = Q3*p3*2.0;
  t12 = Q4*p2*2.0;
  t13 = Q4*p3*2.0;
  A0[0][0] = t11-t12;
  A0[0][1] = t9+t13;
  A0[0][2] = t5+t6-Q3*p1*4.0;
  A0[0][3] = -t3+t8-Q4*p1*4.0;
  A0[1][0] = -t8+t10;
  A0[1][1] = -t5+t7-Q2*p2*4.0;
  A0[1][2] = t4+t13;
  A0[1][3] = t2+t11-Q4*p2*4.0;
  A0[2][0] = t6-t7;
  A0[2][1] = t3+t10-Q2*p3*4.0;
  A0[2][2] = -t2+t12-Q3*p3*4.0;
  A0[2][3] = t4+t9;
