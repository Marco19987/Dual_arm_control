classdef Helper
   methods(Static)
       function qdot = quaternion_propagation(q, omega)
            % quaternion representation q = [eta epsilon], i.e., q = [qw qx qy qz]
            eta = q(1);
            epsilon = q(2:end);
            eta_dot = -0.5*epsilon'* omega;
            epsilon_dot = 0.5*(eta*eye(3)-skew(epsilon))*omega;
            qdot = [eta_dot; epsilon_dot];
      
       end
       function T = transformation_matrix(p,q)
           T = [quat2rotm(q(:)'), p(:); 0 0 0 1];
       end

       function [omega_traj,R_traj] = axis_angle_trajectory(Ri,Rf,points_time,time_vec,Npoints)
            iRf = Ri'*Rf;
            [theta_if, r_if] = tr2angvec(iRf);
            [theta_traj, theta_traj_dot, theta_traj_dot_dot, pp] = quinticpolytraj([0 theta_if],points_time,time_vec);
            
            omega_traj = zeros(3,Npoints); % vector of the desired angular velocities for the frame
            R_traj = zeros(3,3,Npoints);
            for i=1:Npoints
                R_i = Ri*angvec2r(theta_traj(i), r_if);
                omega_i = theta_traj_dot(i) * r_if'; % angular velocity in the initial frame
                omega = R_i * omega_i;            % angular velocity in the world frame
                omega_traj(1:3,i) = omega;        % save the angular trajectory
                R_traj(:,:,i) = R_i;               % save the rotations
            end 


       end

       function q = my_rotm2quat(R)
            
            % Assuming R is a 3x3 matrix
            m00 = R(1, 1);
            m01 = R(1, 2);
            m02 = R(1, 3);
            m10 = R(2, 1);
            m11 = R(2, 2);
            m12 = R(2, 3);
            m20 = R(3, 1);
            m21 = R(3, 2);
            m22 = R(3, 3);

            
            tr = m00 + m11 + m22;
            if (tr > 0) 
              disp("case 1")
              S = sqrt(tr+1.0) * 2;
              qw = 0.25 * S;
              qx = (m21 - m12) / S;
              qy = (m02 - m20) / S; 
              qz = (m10 - m01) / S; 
            elseif(m00 > m11) && (m00 > m22)  
                  disp("case 2")
                  S = sqrt(1.0 + m00 - m11 - m22) * 2;  
                  qw = (m21 - m12) / S;
                  qx = 0.25 * S;
                  qy = (m01 + m10) / S; 
                  qz = (m02 + m20) / S; 
            
            elseif(m11 > m22)  
                disp("case 3")
                  S = sqrt(1.0 + m11 - m00 - m22) * 2; 
                  qw = (m02 - m20) / S;
                  qx = (m01 + m10) / S; 
                  qy = 0.25 * S;
                  qz = (m12 + m21) / S; 
            else  
               disp("case 4")
              S = sqrt(1.0 + m22 - m00 - m11) * 2; 
              qw = (m10 - m01) / S;
              qx = (m02 + m20) / S;
              qy = (m12 + m21) / S;
              qz = 0.25 * S;
            end 
            q = [qw qx qy qz];
       end 

       function R = my_quat2rotm(q)
               q = q(:);
               q = reshape(q,[4 1 size(q,2)]);
               qw = q(1,1,:); qx = q(2,1,:); qy = q(3,1,:); qz = q(4,1,:);
               R = [2*(qw.^2+qx.^2)-1, 2*qx.*qy - 2*qz.*qw, 2*qx.*qz + 2*qy.*qw;
                 2*qx.*qy + 2*qz.*qw, 2*(qw.^2 + qy.^2)-1, 2*qy.*qz - 2*qx.*qw;
                 2*qx.*qz - 2*qy.*qw, 2*qy.*qz + 2*qx.*qw, 2*(qw.^2 + qz.^2)-1];

       end

       function q = quaternion_continuity(qnew,qold)
          qnew = qnew(:);
          qold = qold(:);
          q = qnew;
          tmp = qnew(2:4)' * qold(2:4);
          if (tmp < -0.01)
            q = -q;
          end          

       end

       function q_prod = quaternion_product(q1,q2)
            qw_2 = q2(1); qx_2 = q2(2);  qy_2 = q2(3); qz_2 = q2(4); 
            qw_1 = q1(1); qx_1 = q1(2);  qy_1 = q1(3); qz_1 = q1(4); 
            epsilon_2 = [qx_2,qy_2,qz_2]';
            epsilon_1 = [qx_1,qy_1,qz_1]';
            q_prod(1) = qw_1*qw_2 - epsilon_1'*epsilon_2;
            q_prod(2:4) = qw_1*epsilon_2 + qw_2*epsilon_1 + skew(epsilon_1)*epsilon_2;
       end

       function q_inv = quaternion_inverse(q)
            % Extract the scalar and vector parts of the quaternion
            qw = q(1);
            qx = q(2);
            qy = q(3);
            qz = q(4);
        
            % Compute the norm of the quaternion
            norm_q = sqrt(qw^2 + qx^2 + qy^2 + qz^2);
        
            % Compute the inverse of the quaternion
            q_inv = [qw, -qx, -qy, -qz] / norm_q^2;
       end
       function q = normalize_quaternion(q)
            q = q/norm(q);
       end


         
   end
end