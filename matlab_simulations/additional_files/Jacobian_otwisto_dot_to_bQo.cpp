  t2 = bQo1*bg1*2.0;
  t3 = bQo1*bg2*2.0;
  t4 = bQo2*bg1*2.0;
  t5 = bQo1*bg3*2.0;
  t6 = bQo2*bg2*2.0;
  t7 = bQo3*bg1*2.0;
  t8 = bQo2*bg3*2.0;
  t9 = bQo3*bg2*2.0;
  t10 = bQo4*bg1*2.0;
  t11 = bQo3*bg3*2.0;
  t12 = bQo4*bg2*2.0;
  t13 = bQo4*bg3*2.0;
  A0[0][0] = -t11+t12;
  A0[0][1] = t9+t13;
  A0[0][2] = -t5+t6-bQo3*bg1*4.0;
  A0[0][3] = t3+t8-bQo4*bg1*4.0;
  A0[1][0] = t8-t10;
  A0[1][1] = t5+t7-bQo2*bg2*4.0;
  A0[1][2] = t4+t13;
  A0[1][3] = -t2+t11-bQo4*bg2*4.0;
  A0[2][0] = -t6+t7;
  A0[2][1] = -t3+t10-bQo2*bg3*4.0;
  A0[2][2] = t2+t12-bQo3*bg3*4.0;
  A0[2][3] = t4+t9;
