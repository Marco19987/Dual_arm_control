function Jacobian_bpo_dot_to_Q = Jacobian_Rp_to_Q(Q,p)
% Jacobian of the function R(Q)*p, with R(Q) being the 
% rotation matrix corresponding to the quaternion Q and p being a 
% 3D vector,
% with respect to the quaternion Q

Q1 = Q(1,:);
Q2 = Q(2,:);
Q3 = Q(3,:);
Q4 = Q(4,:);
p1 = p(1,:);
p2 = p(2,:);
p3 = p(3,:);
t2 = Q1.*p1.*2.0;
t3 = Q1.*p2.*2.0;
t4 = Q2.*p1.*2.0;
t5 = Q1.*p3.*2.0;
t6 = Q2.*p2.*2.0;
t7 = Q3.*p1.*2.0;
t8 = Q2.*p3.*2.0;
t9 = Q3.*p2.*2.0;
t10 = Q4.*p1.*2.0;
t11 = Q3.*p3.*2.0;
t12 = Q4.*p2.*2.0;
t13 = Q4.*p3.*2.0;
Jacobian_bpo_dot_to_Q = reshape([t11-t12,-t8+t10,t6-t7,t9+t13,-t5+t7-Q2.*p2.*4.0,t3+t10-Q2.*p3.*4.0,t5+t6-Q3.*p1.*4.0,t4+t13,-t2+t12-Q3.*p3.*4.0,-t3+t8-Q4.*p1.*4.0,t2+t11-Q4.*p2.*4.0,t4+t9],[3,4]);
end
