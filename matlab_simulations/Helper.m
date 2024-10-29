classdef Helper
   methods(Static)
       function qdot = quaternion_propagation(q, omega)
            % quaternion representation q = [eta epsilon], i.e., q = [qw qx qy qz]
            eta = q(1);
            epsilon = q(2:end);
            eta_dot = -0.5*epsilon'* omega;
            epsilon_dot = 0.5*(eta*eye(3)-skew(epsilon))* omega;
            qdot = [eta_dot; epsilon_dot];
      
       end
       function T = transformation_matrix(p,q)
           T = [quat2rotm(q(:)'), p(:); 0 0 0 1];
       end
   end
end