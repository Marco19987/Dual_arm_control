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
   end
end