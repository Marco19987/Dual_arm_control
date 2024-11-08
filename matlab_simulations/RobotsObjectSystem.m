classdef RobotsObjectSystem < SimpleSystem
    % system describing the interaction between 2 robots and one object
    % the input to the system are the exerted wrenches by the robots
    % applied in the grasp frames and expressed in the same frame
    % the output of the system are the measured poses of the object 
    % the state is represented by the pose of the object in the inertial
    % frame an the twist
    % pose = [px py pz qx qy qz qw]';
    % twist = [vx vy vz omegax omegay omegaz]';

    properties      

        Bm     % object inertia matrix
        bg     % gravity vector in the base frame 
        opg1   % position of the grasp frame of robot1 expressed in the object frame
        opg2   % position of the grasp frame of robot2 expressed in the object frame
        oRg1   % rotation matrix of the grasp frame of robot1 expressed in the object frame
        oRg2   % rotation matrix of the grasp frame of robot2 expressed in the object frame

        n_pose_measures      % number pose estimators 
                             % => there will be 2*n_pose_measures measures of the 
                             % object pose (for each robot we have n_pose_measures)
                             % the poses measured by robot_i are expressed
                             % in the grasp_i frame for i=1,2
                            
        W      % grasp matrix
        Rbar   % rotation matrixs between grasp frames and object frame

        b1pg1   % position of grasp1 frame in the robot 1 base frame
        b1Qg1   % quaternion of grasp1 frame in the robot 1 base frame
        b2pg2   % position of grasp2 frame in the robot 2 base frame
        b2Qg2   % quaternion of grasp2 frame in the robot 2 base frame
        % the above poses of grasp frames change in the time due to the
        % robot movement. They are assumed to be updated from outside.

        b1pb2   % robot 2 base frame position wrt the robot 1 base frame
        b1Qb2   % robot 2 base frame quaternion wrt the robot 1 base frame
        % This pose can be added as state variable of the filter for 
        % a further refinement

        bpb1   % position of robot 1 wrt the base frame into wich express the object pose and twist
        bQb1   % quaternion of robot 1 wrt the base frame into wich express the object pose and twist
        % assumed to be constant

    end
    methods
        function obj = RobotsObjectSystem(state, sizeState, sizeOutput,SampleTime, Bm,bg,opg1,opg2,oRg1,oRg2,n_pose_measures ...
                                            ,b1pg1, b1Qg1,b2pg2,b2Qg2,b1pb2,b1Qb2,bpb1,bQb1)
            % Call the constructor of the superclass
            obj@SimpleSystem(state, sizeState, sizeOutput, SampleTime);
            
            obj.Bm = Bm;
            obj.bg = bg;
            obj.opg1 = opg1;
            obj.opg2 = opg2;
            obj.oRg1 = oRg1;
            obj.oRg2 = oRg2;
            obj.n_pose_measures = n_pose_measures;
            obj.update_grasp_matrix()
            obj.update_Rbar()

            obj.b1pg1 = b1pg1;
            obj.b1Qg1 = b1Qg1;
            obj.b2pg2 = b2pg2;
            obj.b2Qg2 = b2Qg2;
            obj.b1pb2 = b1pb2;
            obj.b1Qb2 = b1Qb2;

            obj.bpb1 = bpb1;
            obj.bQb1 = bQb1;
        end
   
        function update_b1Tg1(obj,b1pg1,b1Qg1)
            obj.b1pg1 = b1pg1;
            obj.b1Qg1 = b1Qg1;
        end  

        function update_b2Tg2(obj,b2pg2,b2Qg2)
            obj.b2pg2 = b2pg2;
            obj.b2Qg2 = b2Qg2;
        end  

        function update_grasp_matrix(obj)
            % compute grasp matrix 
            Wg1 = [eye(3), zeros(3); -skew(obj.opg1)', eye(3)];
            Wg2 = [eye(3), zeros(3); -skew(obj.opg2)', eye(3)];
            obj.W = [Wg1,Wg2];
        end 

        function update_Rbar(obj)
            obj.Rbar = blkdiag(obj.oRg1,obj.oRg1,obj.oRg2,obj.oRg2);
        end

        function xdot = eval_xdot(obj, x, u)
            x(4:7) = norm(x(4:7));
            % implementation of xdot = f(x,u)
            bvo = x(8:10);      % object's linear velocity in the base frame
            bomegao = x(11:13); % object's angular velocity in the base frame
            bQo = x(4:7);       % object's quaternion in the base frame
            g1hg1 = u(1:6);     % robot1's grasp wrench in the grasp frame 1
            g2hg2 = u(7:12);    % robot2's grasp wrench in the grasp frame 2
            oh = obj.W * obj.Rbar * [g1hg1;g2hg2];  % resulting wrench in the object frame
            bRo = quat2rotm(bQo');

            xdot(1:3) = bvo;
            xdot(4:7) = Helper.quaternion_propagation(bQo,bomegao);
            xdot(8:13) = blkdiag(bRo,bRo)*(obj.Bm\oh) + [obj.bg;0;0;0];
        end
        
        function newState = state_fcn(obj, x, u)
            % discretized version
            % xk+1 = xk + SampleTime * xdotk
            newState = x + obj.SampleTime * obj.eval_xdot(x, u)';         
        end
        
        function output = output_fcn(obj, x, u)
           % in the output there are the measures coming from the pose
           % estimators
           % there is only a transformation involved in the relation 
            
           output = zeros(7*obj.n_pose_measures*2,1); 

           b1Tg1 = Helper.transformation_matrix(obj.b1pg1, obj.b1Qg1);
           b2Tg2 = Helper.transformation_matrix(obj.b2pg2, obj.b2Qg2);
           
           b1Tb2 = Helper.transformation_matrix(obj.b1pb2, obj.b1Qb2);
           bTb1 = Helper.transformation_matrix(obj.bpb1, obj.bQb1);
           
           bTo = Helper.transformation_matrix(x(1:3), x(4:7));
            
           g1To = inv(b1Tg1) * inv(bTb1) * bTo; % measured object pose from robot 1 with the k-th pose estimator in the grasp1 frame
           g2To = inv(b2Tg2) * inv(b1Tb2) * inv(bTb1) * bTo; % measured object pose from robot 1 with the k-th pose estimator in the grasp1 frame

           g1qo = [g1To(1:3,4);rotm2quat(g1To(1:3,1:3))']; 
           g2qo = [g2To(1:3,4);rotm2quat(g2To(1:3,1:3))'];
            
           output(1:7*obj.n_pose_measures) = repmat(g1qo,obj.n_pose_measures,1);
           output(7*obj.n_pose_measures+1:end) = repmat(g2qo,obj.n_pose_measures,1);

            
        end
        
        function jacobian = jacob_state_fcn(obj, x, u)
            % Override the Jacobian of the state transition function
            bQo = x(4:7);
            bomegao = x(8:10);
            jacobian = zeros(length(x));

            % linear velocity term
            jacobian(1:3,8:10) = eye(3); 
            
            % quaternion dot term
            jacobian_fq_q = 0.5*[1 bomegao'; bomegao, -skew(bomegao)]; %  jacobian of 0.5*E[omega]*Q wrt Q
            jacobian_fq_omega = 0.5*[ bQo(2),  bQo(3),  bQo(4); % jacobian of 0.5*E[omega]*Q wrt omega
                       bQo(1),  bQo(4), -bQo(3);
                      -bQo(4),  bQo(1),  bQo(2);
                       bQo(3), -bQo(2),  bQo(1)];

            jacobian(4:7,4:7) = jacobian_fq_q; 
            jacobian(4:7,8:10) = jacobian_fq_omega;

            % acceleration term 
            g1hg1 = u(1:6);    
            g2hg2 = u(7:12);   
            oh = obj.W * obj.Rbar * [g1hg1;g2hg2];  
            jacobian(8:13,4:7) = obj.jacob_dynamics_to_quaternion(bQo,obj.Bm\oh);
            
            jacobian = eye(length(x))-obj.SampleTime*jacobian;

        end
        
        function jacobian = jacob_output_fcn(obj, x, u)
            % Override the Jacobian of the output function

            b1Tg1 = Helper.transformation_matrix(obj.b1pg1, obj.b1Qg1);
            b2Tg2 = Helper.transformation_matrix(obj.b2pg2, obj.b2Qg2);
           
            b1Tb2 = Helper.transformation_matrix(obj.b1pb2, obj.b1Qb2);
            bTb1 = Helper.transformation_matrix(obj.bpb1, obj.bQb1);

            %bTo = Helper.transformation_matrix(x(1:3), x(4:7));
            bQo = x(4:7);

            jacobian = zeros(7*2*obj.n_pose_measures,length(x));

            % jacobian measures from robot 1
            g1Tb = inv(b1Tg1) * inv(bTb1);
            Jp_1 = obj.jacobian_output_to_position(g1Tb);   %output position jacobian robot 1
            JQ_1 = obj.jacobian_output_to_quaternion(g1Tb,bQo);  %output quaternion jacobian robot 1
            output_jacobian_1_kth = [Jp_1, JQ_1, zeros(7,3), zeros(7,3)];
            jacobian(1:7*obj.n_pose_measures,:) = repmat(output_jacobian_1_kth,obj.n_pose_measures,1);
            
            % jacobian measures from robot 1
            g2Tb = inv(b2Tg2) * inv(b1Tb2) * inv(bTb1);
            Jp_2 = obj.jacobian_output_to_position(g2Tb);   %output position jacobian robot 1
            JQ_2 = obj.jacobian_output_to_quaternion(g2Tb,bQo);  %output quaternion jacobian robot 1
            output_jacobian_2_kth = [Jp_2, JQ_2, zeros(7,3), zeros(7,3)];
            jacobian(7*obj.n_pose_measures+1:end,:) = repmat(output_jacobian_2_kth,obj.n_pose_measures,1);

        end

        function jacobian = jacob_dynamics_to_quaternion(obj,q,a)
            % this function computes the jacobian of the term relative
            % to the accelerations wrt the quaternion variable
            % the vector "a" should be the result of the product (obj.Bm\oh)

            qw = q(1); qx = q(2); qy = q(3); qz = q(4);
            a1 = a(1);a2 = a(2);a3 = a(3);a4 = a(4);a5 = a(5);a6 = a(6);

            jacobian = [2*a3*qy - 2*a2*qz, 2*a2*qy + 2*a3*qz, 2*a3*qw + 2*a2*qx - 4*a1*qy, 2*a3*qx - 2*a2*qw - 4*a1*qz;
                        2*a1*qz - 2*a3*qx, 2*a1*qy - 4*a2*qx - 2*a3*qw,           2*a1*qx + 2*a3*qz, 2*a1*qw + 2*a3*qy - 4*a2*qz;
                        2*a2*qx - 2*a1*qy, 2*a2*qw - 4*a3*qx + 2*a1*qz, 2*a2*qz - 4*a3*qy - 2*a1*qw,           2*a1*qx + 2*a2*qy;
                        2*a6*qy - 2*a5*qz,           2*a5*qy + 2*a6*qz, 2*a6*qw + 2*a5*qx - 4*a4*qy, 2*a6*qx - 2*a5*qw - 4*a4*qz;
                        2*a4*qz - 2*a6*qx, 2*a4*qy - 4*a5*qx - 2*a6*qw,           2*a4*qx + 2*a6*qz, 2*a4*qw + 2*a6*qy - 4*a5*qz;
                        2*a5*qx - 2*a4*qy, 2*a5*qw - 4*a6*qx + 2*a4*qz, 2*a5*qz - 4*a6*qy - 2*a4*qw,           2*a4*qx + 2*a5*qy];

        end 

        function jacobian = jacobian_output_to_position(obj,gkTb)
            % depends only from the rotation matrix between the base frame
            % and the k-th grasp frame 
            jacobian = [gkTb(1:3,1:3);zeros(4,3)];
        end 

        function jacobian = jacobian_output_to_quaternion(obj,gkTb,q)
            % depends only from the rotation matrix between the base frame
            % and the k-th grasp frame 
            qw = q(1); qx = q(2); qy = q(3); qz = q(4);

            gkTb1_1 = gkTb(1,1); gkTb1_2 = gkTb(1,2); gkTb1_3 = gkTb(1,3);
            gkTb2_1 = gkTb(2,1); gkTb2_2 = gkTb(2,2); gkTb2_3 = gkTb(2,3);
            gkTb3_1 = gkTb(3,1); gkTb3_2 = gkTb(3,2); gkTb3_3 = gkTb(3,3);

            jacobian = zeros(7,4);                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                            
            jacobian(4:7,1:4) = [
                   (2*gkTb2_3*qx - 2*gkTb3_2*qx - 2*gkTb1_3*qy + 2*gkTb3_1*qy + 2*gkTb1_2*qz - 2*gkTb2_1*qz)/(4*(gkTb1_2*(2*qw*qz + 2*qx*qy) - gkTb2_2*(2*qx^2 + 2*qz^2 - 1) - gkTb1_1*(2*qy^2 + 2*qz^2 - 1) - gkTb3_3*(2*qx^2 + 2*qy^2 - 1) - gkTb1_3*(2*qw*qy - 2*qx*qz) - gkTb2_1*(2*qw*qz - 2*qx*qy) + gkTb2_3*(2*qw*qx + 2*qy*qz) + gkTb3_1*(2*qw*qy + 2*qx*qz) - gkTb3_2*(2*qw*qx - 2*qy*qz) + 1)^(1/2)),                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                         (2*gkTb2_3*qw - 2*gkTb3_2*qw - 4*gkTb2_2*qx - 4*gkTb3_3*qx + 2*gkTb1_2*qy + 2*gkTb2_1*qy + 2*gkTb1_3*qz + 2*gkTb3_1*qz)/(4*(gkTb1_2*(2*qw*qz + 2*qx*qy) - gkTb2_2*(2*qx^2 + 2*qz^2 - 1) - gkTb1_1*(2*qy^2 + 2*qz^2 - 1) - gkTb3_3*(2*qx^2 + 2*qy^2 - 1) - gkTb1_3*(2*qw*qy - 2*qx*qz) - gkTb2_1*(2*qw*qz - 2*qx*qy) + gkTb2_3*(2*qw*qx + 2*qy*qz) + gkTb3_1*(2*qw*qy + 2*qx*qz) - gkTb3_2*(2*qw*qx - 2*qy*qz) + 1)^(1/2)),                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                         (2*gkTb3_1*qw - 2*gkTb1_3*qw + 2*gkTb1_2*qx + 2*gkTb2_1*qx - 4*gkTb1_1*qy - 4*gkTb3_3*qy + 2*gkTb2_3*qz + 2*gkTb3_2*qz)/(4*(gkTb1_2*(2*qw*qz + 2*qx*qy) - gkTb2_2*(2*qx^2 + 2*qz^2 - 1) - gkTb1_1*(2*qy^2 + 2*qz^2 - 1) - gkTb3_3*(2*qx^2 + 2*qy^2 - 1) - gkTb1_3*(2*qw*qy - 2*qx*qz) - gkTb2_1*(2*qw*qz - 2*qx*qy) + gkTb2_3*(2*qw*qx + 2*qy*qz) + gkTb3_1*(2*qw*qy + 2*qx*qz) - gkTb3_2*(2*qw*qx - 2*qy*qz) + 1)^(1/2)),                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                         (2*gkTb1_2*qw - 2*gkTb2_1*qw + 2*gkTb1_3*qx + 2*gkTb3_1*qx + 2*gkTb2_3*qy + 2*gkTb3_2*qy - 4*gkTb1_1*qz - 4*gkTb2_2*qz)/(4*(gkTb1_2*(2*qw*qz + 2*qx*qy) - gkTb2_2*(2*qx^2 + 2*qz^2 - 1) - gkTb1_1*(2*qy^2 + 2*qz^2 - 1) - gkTb3_3*(2*qx^2 + 2*qy^2 - 1) - gkTb1_3*(2*qw*qy - 2*qx*qz) - gkTb2_1*(2*qw*qz - 2*qx*qy) + gkTb2_3*(2*qw*qx + 2*qy*qz) + gkTb3_1*(2*qw*qy + 2*qx*qz) - gkTb3_2*(2*qw*qx - 2*qy*qz) + 1)^(1/2));
                   (2*gkTb2_2*qx + 2*gkTb3_3*qx - 2*gkTb2_1*qy - 2*gkTb3_1*qz)/(2*(gkTb1_2*(2*qw*qz + 2*qx*qy) - gkTb2_2*(2*qx^2 + 2*qz^2 - 1) - gkTb1_1*(2*qy^2 + 2*qz^2 - 1) - gkTb3_3*(2*qx^2 + 2*qy^2 - 1) - gkTb1_3*(2*qw*qy - 2*qx*qz) - gkTb2_1*(2*qw*qz - 2*qx*qy) + gkTb2_3*(2*qw*qx + 2*qy*qz) + gkTb3_1*(2*qw*qy + 2*qx*qz) - gkTb3_2*(2*qw*qx - 2*qy*qz) + 1)^(1/2)) - ((gkTb2_3*(2*qx^2 + 2*qy^2 - 1) - gkTb3_2*(2*qx^2 + 2*qz^2 - 1) - gkTb2_1*(2*qw*qy + 2*qx*qz) + gkTb2_2*(2*qw*qx - 2*qy*qz) - gkTb3_1*(2*qw*qz - 2*qx*qy) + gkTb3_3*(2*qw*qx + 2*qy*qz))*(2*gkTb2_3*qx - 2*gkTb3_2*qx - 2*gkTb1_3*qy + 2*gkTb3_1*qy + 2*gkTb1_2*qz - 2*gkTb2_1*qz))/(4*(gkTb1_2*(2*qw*qz + 2*qx*qy) - gkTb2_2*(2*qx^2 + 2*qz^2 - 1) - gkTb1_1*(2*qy^2 + 2*qz^2 - 1) - gkTb3_3*(2*qx^2 + 2*qy^2 - 1) - gkTb1_3*(2*qw*qy - 2*qx*qz) - gkTb2_1*(2*qw*qz - 2*qx*qy) + gkTb2_3*(2*qw*qx + 2*qy*qz) + gkTb3_1*(2*qw*qy + 2*qx*qz) - gkTb3_2*(2*qw*qx - 2*qy*qz) + 1)^(3/2)), (2*gkTb2_2*qw + 2*gkTb3_3*qw + 4*gkTb2_3*qx - 4*gkTb3_2*qx + 2*gkTb3_1*qy - 2*gkTb2_1*qz)/(2*(gkTb1_2*(2*qw*qz + 2*qx*qy) - gkTb2_2*(2*qx^2 + 2*qz^2 - 1) - gkTb1_1*(2*qy^2 + 2*qz^2 - 1) - gkTb3_3*(2*qx^2 + 2*qy^2 - 1) - gkTb1_3*(2*qw*qy - 2*qx*qz) - gkTb2_1*(2*qw*qz - 2*qx*qy) + gkTb2_3*(2*qw*qx + 2*qy*qz) + gkTb3_1*(2*qw*qy + 2*qx*qz) - gkTb3_2*(2*qw*qx - 2*qy*qz) + 1)^(1/2)) - ((gkTb2_3*(2*qx^2 + 2*qy^2 - 1) - gkTb3_2*(2*qx^2 + 2*qz^2 - 1) - gkTb2_1*(2*qw*qy + 2*qx*qz) + gkTb2_2*(2*qw*qx - 2*qy*qz) - gkTb3_1*(2*qw*qz - 2*qx*qy) + gkTb3_3*(2*qw*qx + 2*qy*qz))*(2*gkTb2_3*qw - 2*gkTb3_2*qw - 4*gkTb2_2*qx - 4*gkTb3_3*qx + 2*gkTb1_2*qy + 2*gkTb2_1*qy + 2*gkTb1_3*qz + 2*gkTb3_1*qz))/(4*(gkTb1_2*(2*qw*qz + 2*qx*qy) - gkTb2_2*(2*qx^2 + 2*qz^2 - 1) - gkTb1_1*(2*qy^2 + 2*qz^2 - 1) - gkTb3_3*(2*qx^2 + 2*qy^2 - 1) - gkTb1_3*(2*qw*qy - 2*qx*qz) - gkTb2_1*(2*qw*qz - 2*qx*qy) + gkTb2_3*(2*qw*qx + 2*qy*qz) + gkTb3_1*(2*qw*qy + 2*qx*qz) - gkTb3_2*(2*qw*qx - 2*qy*qz) + 1)^(3/2)),                (2*gkTb3_1*qx - 2*gkTb2_1*qw + 4*gkTb2_3*qy - 2*gkTb2_2*qz + 2*gkTb3_3*qz)/(2*(gkTb1_2*(2*qw*qz + 2*qx*qy) - gkTb2_2*(2*qx^2 + 2*qz^2 - 1) - gkTb1_1*(2*qy^2 + 2*qz^2 - 1) - gkTb3_3*(2*qx^2 + 2*qy^2 - 1) - gkTb1_3*(2*qw*qy - 2*qx*qz) - gkTb2_1*(2*qw*qz - 2*qx*qy) + gkTb2_3*(2*qw*qx + 2*qy*qz) + gkTb3_1*(2*qw*qy + 2*qx*qz) - gkTb3_2*(2*qw*qx - 2*qy*qz) + 1)^(1/2)) - ((gkTb2_3*(2*qx^2 + 2*qy^2 - 1) - gkTb3_2*(2*qx^2 + 2*qz^2 - 1) - gkTb2_1*(2*qw*qy + 2*qx*qz) + gkTb2_2*(2*qw*qx - 2*qy*qz) - gkTb3_1*(2*qw*qz - 2*qx*qy) + gkTb3_3*(2*qw*qx + 2*qy*qz))*(2*gkTb3_1*qw - 2*gkTb1_3*qw + 2*gkTb1_2*qx + 2*gkTb2_1*qx - 4*gkTb1_1*qy - 4*gkTb3_3*qy + 2*gkTb2_3*qz + 2*gkTb3_2*qz))/(4*(gkTb1_2*(2*qw*qz + 2*qx*qy) - gkTb2_2*(2*qx^2 + 2*qz^2 - 1) - gkTb1_1*(2*qy^2 + 2*qz^2 - 1) - gkTb3_3*(2*qx^2 + 2*qy^2 - 1) - gkTb1_3*(2*qw*qy - 2*qx*qz) - gkTb2_1*(2*qw*qz - 2*qx*qy) + gkTb2_3*(2*qw*qx + 2*qy*qz) + gkTb3_1*(2*qw*qy + 2*qx*qz) - gkTb3_2*(2*qw*qx - 2*qy*qz) + 1)^(3/2)),              - (2*gkTb3_1*qw + 2*gkTb2_1*qx + 2*gkTb2_2*qy - 2*gkTb3_3*qy + 4*gkTb3_2*qz)/(2*(gkTb1_2*(2*qw*qz + 2*qx*qy) - gkTb2_2*(2*qx^2 + 2*qz^2 - 1) - gkTb1_1*(2*qy^2 + 2*qz^2 - 1) - gkTb3_3*(2*qx^2 + 2*qy^2 - 1) - gkTb1_3*(2*qw*qy - 2*qx*qz) - gkTb2_1*(2*qw*qz - 2*qx*qy) + gkTb2_3*(2*qw*qx + 2*qy*qz) + gkTb3_1*(2*qw*qy + 2*qx*qz) - gkTb3_2*(2*qw*qx - 2*qy*qz) + 1)^(1/2)) - ((gkTb2_3*(2*qx^2 + 2*qy^2 - 1) - gkTb3_2*(2*qx^2 + 2*qz^2 - 1) - gkTb2_1*(2*qw*qy + 2*qx*qz) + gkTb2_2*(2*qw*qx - 2*qy*qz) - gkTb3_1*(2*qw*qz - 2*qx*qy) + gkTb3_3*(2*qw*qx + 2*qy*qz))*(2*gkTb1_2*qw - 2*gkTb2_1*qw + 2*gkTb1_3*qx + 2*gkTb3_1*qx + 2*gkTb2_3*qy + 2*gkTb3_2*qy - 4*gkTb1_1*qz - 4*gkTb2_2*qz))/(4*(gkTb1_2*(2*qw*qz + 2*qx*qy) - gkTb2_2*(2*qx^2 + 2*qz^2 - 1) - gkTb1_1*(2*qy^2 + 2*qz^2 - 1) - gkTb3_3*(2*qx^2 + 2*qy^2 - 1) - gkTb1_3*(2*qw*qy - 2*qx*qz) - gkTb2_1*(2*qw*qz - 2*qx*qy) + gkTb2_3*(2*qw*qx + 2*qy*qz) + gkTb3_1*(2*qw*qy + 2*qx*qz) - gkTb3_2*(2*qw*qx - 2*qy*qz) + 1)^(3/2));
                   ((gkTb1_3*(2*qx^2 + 2*qy^2 - 1) - gkTb3_1*(2*qy^2 + 2*qz^2 - 1) - gkTb1_1*(2*qw*qy + 2*qx*qz) + gkTb1_2*(2*qw*qx - 2*qy*qz) + gkTb3_2*(2*qw*qz + 2*qx*qy) - gkTb3_3*(2*qw*qy - 2*qx*qz))*(2*gkTb2_3*qx - 2*gkTb3_2*qx - 2*gkTb1_3*qy + 2*gkTb3_1*qy + 2*gkTb1_2*qz - 2*gkTb2_1*qz))/(4*(gkTb1_2*(2*qw*qz + 2*qx*qy) - gkTb2_2*(2*qx^2 + 2*qz^2 - 1) - gkTb1_1*(2*qy^2 + 2*qz^2 - 1) - gkTb3_3*(2*qx^2 + 2*qy^2 - 1) - gkTb1_3*(2*qw*qy - 2*qx*qz) - gkTb2_1*(2*qw*qz - 2*qx*qy) + gkTb2_3*(2*qw*qx + 2*qy*qz) + gkTb3_1*(2*qw*qy + 2*qx*qz) - gkTb3_2*(2*qw*qx - 2*qy*qz) + 1)^(3/2)) - (2*gkTb1_2*qx - 2*gkTb1_1*qy - 2*gkTb3_3*qy + 2*gkTb3_2*qz)/(2*(gkTb1_2*(2*qw*qz + 2*qx*qy) - gkTb2_2*(2*qx^2 + 2*qz^2 - 1) - gkTb1_1*(2*qy^2 + 2*qz^2 - 1) - gkTb3_3*(2*qx^2 + 2*qy^2 - 1) - gkTb1_3*(2*qw*qy - 2*qx*qz) - gkTb2_1*(2*qw*qz - 2*qx*qy) + gkTb2_3*(2*qw*qx + 2*qy*qz) + gkTb3_1*(2*qw*qy + 2*qx*qz) - gkTb3_2*(2*qw*qx - 2*qy*qz) + 1)^(1/2)),                ((gkTb1_3*(2*qx^2 + 2*qy^2 - 1) - gkTb3_1*(2*qy^2 + 2*qz^2 - 1) - gkTb1_1*(2*qw*qy + 2*qx*qz) + gkTb1_2*(2*qw*qx - 2*qy*qz) + gkTb3_2*(2*qw*qz + 2*qx*qy) - gkTb3_3*(2*qw*qy - 2*qx*qz))*(2*gkTb2_3*qw - 2*gkTb3_2*qw - 4*gkTb2_2*qx - 4*gkTb3_3*qx + 2*gkTb1_2*qy + 2*gkTb2_1*qy + 2*gkTb1_3*qz + 2*gkTb3_1*qz))/(4*(gkTb1_2*(2*qw*qz + 2*qx*qy) - gkTb2_2*(2*qx^2 + 2*qz^2 - 1) - gkTb1_1*(2*qy^2 + 2*qz^2 - 1) - gkTb3_3*(2*qx^2 + 2*qy^2 - 1) - gkTb1_3*(2*qw*qy - 2*qx*qz) - gkTb2_1*(2*qw*qz - 2*qx*qy) + gkTb2_3*(2*qw*qx + 2*qy*qz) + gkTb3_1*(2*qw*qy + 2*qx*qz) - gkTb3_2*(2*qw*qx - 2*qy*qz) + 1)^(3/2)) - (2*gkTb1_2*qw + 4*gkTb1_3*qx + 2*gkTb3_2*qy - 2*gkTb1_1*qz + 2*gkTb3_3*qz)/(2*(gkTb1_2*(2*qw*qz + 2*qx*qy) - gkTb2_2*(2*qx^2 + 2*qz^2 - 1) - gkTb1_1*(2*qy^2 + 2*qz^2 - 1) - gkTb3_3*(2*qx^2 + 2*qy^2 - 1) - gkTb1_3*(2*qw*qy - 2*qx*qz) - gkTb2_1*(2*qw*qz - 2*qx*qy) + gkTb2_3*(2*qw*qx + 2*qy*qz) + gkTb3_1*(2*qw*qy + 2*qx*qz) - gkTb3_2*(2*qw*qx - 2*qy*qz) + 1)^(1/2)), (2*gkTb1_1*qw + 2*gkTb3_3*qw - 2*gkTb3_2*qx - 4*gkTb1_3*qy + 4*gkTb3_1*qy + 2*gkTb1_2*qz)/(2*(gkTb1_2*(2*qw*qz + 2*qx*qy) - gkTb2_2*(2*qx^2 + 2*qz^2 - 1) - gkTb1_1*(2*qy^2 + 2*qz^2 - 1) - gkTb3_3*(2*qx^2 + 2*qy^2 - 1) - gkTb1_3*(2*qw*qy - 2*qx*qz) - gkTb2_1*(2*qw*qz - 2*qx*qy) + gkTb2_3*(2*qw*qx + 2*qy*qz) + gkTb3_1*(2*qw*qy + 2*qx*qz) - gkTb3_2*(2*qw*qx - 2*qy*qz) + 1)^(1/2)) + ((gkTb1_3*(2*qx^2 + 2*qy^2 - 1) - gkTb3_1*(2*qy^2 + 2*qz^2 - 1) - gkTb1_1*(2*qw*qy + 2*qx*qz) + gkTb1_2*(2*qw*qx - 2*qy*qz) + gkTb3_2*(2*qw*qz + 2*qx*qy) - gkTb3_3*(2*qw*qy - 2*qx*qz))*(2*gkTb3_1*qw - 2*gkTb1_3*qw + 2*gkTb1_2*qx + 2*gkTb2_1*qx - 4*gkTb1_1*qy - 4*gkTb3_3*qy + 2*gkTb2_3*qz + 2*gkTb3_2*qz))/(4*(gkTb1_2*(2*qw*qz + 2*qx*qy) - gkTb2_2*(2*qx^2 + 2*qz^2 - 1) - gkTb1_1*(2*qy^2 + 2*qz^2 - 1) - gkTb3_3*(2*qx^2 + 2*qy^2 - 1) - gkTb1_3*(2*qw*qy - 2*qx*qz) - gkTb2_1*(2*qw*qz - 2*qx*qy) + gkTb2_3*(2*qw*qx + 2*qy*qz) + gkTb3_1*(2*qw*qy + 2*qx*qz) - gkTb3_2*(2*qw*qx - 2*qy*qz) + 1)^(3/2)),                (2*gkTb1_1*qx - 2*gkTb3_2*qw - 2*gkTb3_3*qx + 2*gkTb1_2*qy + 4*gkTb3_1*qz)/(2*(gkTb1_2*(2*qw*qz + 2*qx*qy) - gkTb2_2*(2*qx^2 + 2*qz^2 - 1) - gkTb1_1*(2*qy^2 + 2*qz^2 - 1) - gkTb3_3*(2*qx^2 + 2*qy^2 - 1) - gkTb1_3*(2*qw*qy - 2*qx*qz) - gkTb2_1*(2*qw*qz - 2*qx*qy) + gkTb2_3*(2*qw*qx + 2*qy*qz) + gkTb3_1*(2*qw*qy + 2*qx*qz) - gkTb3_2*(2*qw*qx - 2*qy*qz) + 1)^(1/2)) + ((gkTb1_3*(2*qx^2 + 2*qy^2 - 1) - gkTb3_1*(2*qy^2 + 2*qz^2 - 1) - gkTb1_1*(2*qw*qy + 2*qx*qz) + gkTb1_2*(2*qw*qx - 2*qy*qz) + gkTb3_2*(2*qw*qz + 2*qx*qy) - gkTb3_3*(2*qw*qy - 2*qx*qz))*(2*gkTb1_2*qw - 2*gkTb2_1*qw + 2*gkTb1_3*qx + 2*gkTb3_1*qx + 2*gkTb2_3*qy + 2*gkTb3_2*qy - 4*gkTb1_1*qz - 4*gkTb2_2*qz))/(4*(gkTb1_2*(2*qw*qz + 2*qx*qy) - gkTb2_2*(2*qx^2 + 2*qz^2 - 1) - gkTb1_1*(2*qy^2 + 2*qz^2 - 1) - gkTb3_3*(2*qx^2 + 2*qy^2 - 1) - gkTb1_3*(2*qw*qy - 2*qx*qz) - gkTb2_1*(2*qw*qz - 2*qx*qy) + gkTb2_3*(2*qw*qx + 2*qy*qz) + gkTb3_1*(2*qw*qy + 2*qx*qz) - gkTb3_2*(2*qw*qx - 2*qy*qz) + 1)^(3/2));
                  -(2*gkTb1_3*qx + 2*gkTb2_3*qy - 2*gkTb1_1*qz - 2*gkTb2_2*qz)/(2*(gkTb1_2*(2*qw*qz + 2*qx*qy) - gkTb2_2*(2*qx^2 + 2*qz^2 - 1) - gkTb1_1*(2*qy^2 + 2*qz^2 - 1) - gkTb3_3*(2*qx^2 + 2*qy^2 - 1) - gkTb1_3*(2*qw*qy - 2*qx*qz) - gkTb2_1*(2*qw*qz - 2*qx*qy) + gkTb2_3*(2*qw*qx + 2*qy*qz) + gkTb3_1*(2*qw*qy + 2*qx*qz) - gkTb3_2*(2*qw*qx - 2*qy*qz) + 1)^(1/2)) - ((gkTb1_2*(2*qx^2 + 2*qz^2 - 1) - gkTb2_1*(2*qy^2 + 2*qz^2 - 1) + gkTb1_1*(2*qw*qz - 2*qx*qy) - gkTb1_3*(2*qw*qx + 2*qy*qz) + gkTb2_2*(2*qw*qz + 2*qx*qy) - gkTb2_3*(2*qw*qy - 2*qx*qz))*(2*gkTb2_3*qx - 2*gkTb3_2*qx - 2*gkTb1_3*qy + 2*gkTb3_1*qy + 2*gkTb1_2*qz - 2*gkTb2_1*qz))/(4*(gkTb1_2*(2*qw*qz + 2*qx*qy) - gkTb2_2*(2*qx^2 + 2*qz^2 - 1) - gkTb1_1*(2*qy^2 + 2*qz^2 - 1) - gkTb3_3*(2*qx^2 + 2*qy^2 - 1) - gkTb1_3*(2*qw*qy - 2*qx*qz) - gkTb2_1*(2*qw*qz - 2*qx*qy) + gkTb2_3*(2*qw*qx + 2*qy*qz) + gkTb3_1*(2*qw*qy + 2*qx*qz) - gkTb3_2*(2*qw*qx - 2*qy*qz) + 1)^(3/2)),                (4*gkTb1_2*qx - 2*gkTb1_3*qw - 2*gkTb1_1*qy + 2*gkTb2_2*qy + 2*gkTb2_3*qz)/(2*(gkTb1_2*(2*qw*qz + 2*qx*qy) - gkTb2_2*(2*qx^2 + 2*qz^2 - 1) - gkTb1_1*(2*qy^2 + 2*qz^2 - 1) - gkTb3_3*(2*qx^2 + 2*qy^2 - 1) - gkTb1_3*(2*qw*qy - 2*qx*qz) - gkTb2_1*(2*qw*qz - 2*qx*qy) + gkTb2_3*(2*qw*qx + 2*qy*qz) + gkTb3_1*(2*qw*qy + 2*qx*qz) - gkTb3_2*(2*qw*qx - 2*qy*qz) + 1)^(1/2)) - ((gkTb1_2*(2*qx^2 + 2*qz^2 - 1) - gkTb2_1*(2*qy^2 + 2*qz^2 - 1) + gkTb1_1*(2*qw*qz - 2*qx*qy) - gkTb1_3*(2*qw*qx + 2*qy*qz) + gkTb2_2*(2*qw*qz + 2*qx*qy) - gkTb2_3*(2*qw*qy - 2*qx*qz))*(2*gkTb2_3*qw - 2*gkTb3_2*qw - 4*gkTb2_2*qx - 4*gkTb3_3*qx + 2*gkTb1_2*qy + 2*gkTb2_1*qy + 2*gkTb1_3*qz + 2*gkTb3_1*qz))/(4*(gkTb1_2*(2*qw*qz + 2*qx*qy) - gkTb2_2*(2*qx^2 + 2*qz^2 - 1) - gkTb1_1*(2*qy^2 + 2*qz^2 - 1) - gkTb3_3*(2*qx^2 + 2*qy^2 - 1) - gkTb1_3*(2*qw*qy - 2*qx*qz) - gkTb2_1*(2*qw*qz - 2*qx*qy) + gkTb2_3*(2*qw*qx + 2*qy*qz) + gkTb3_1*(2*qw*qy + 2*qx*qz) - gkTb3_2*(2*qw*qx - 2*qy*qz) + 1)^(3/2)),              - (2*gkTb2_3*qw + 2*gkTb1_1*qx - 2*gkTb2_2*qx + 4*gkTb2_1*qy + 2*gkTb1_3*qz)/(2*(gkTb1_2*(2*qw*qz + 2*qx*qy) - gkTb2_2*(2*qx^2 + 2*qz^2 - 1) - gkTb1_1*(2*qy^2 + 2*qz^2 - 1) - gkTb3_3*(2*qx^2 + 2*qy^2 - 1) - gkTb1_3*(2*qw*qy - 2*qx*qz) - gkTb2_1*(2*qw*qz - 2*qx*qy) + gkTb2_3*(2*qw*qx + 2*qy*qz) + gkTb3_1*(2*qw*qy + 2*qx*qz) - gkTb3_2*(2*qw*qx - 2*qy*qz) + 1)^(1/2)) - ((gkTb1_2*(2*qx^2 + 2*qz^2 - 1) - gkTb2_1*(2*qy^2 + 2*qz^2 - 1) + gkTb1_1*(2*qw*qz - 2*qx*qy) - gkTb1_3*(2*qw*qx + 2*qy*qz) + gkTb2_2*(2*qw*qz + 2*qx*qy) - gkTb2_3*(2*qw*qy - 2*qx*qz))*(2*gkTb3_1*qw - 2*gkTb1_3*qw + 2*gkTb1_2*qx + 2*gkTb2_1*qx - 4*gkTb1_1*qy - 4*gkTb3_3*qy + 2*gkTb2_3*qz + 2*gkTb3_2*qz))/(4*(gkTb1_2*(2*qw*qz + 2*qx*qy) - gkTb2_2*(2*qx^2 + 2*qz^2 - 1) - gkTb1_1*(2*qy^2 + 2*qz^2 - 1) - gkTb3_3*(2*qx^2 + 2*qy^2 - 1) - gkTb1_3*(2*qw*qy - 2*qx*qz) - gkTb2_1*(2*qw*qz - 2*qx*qy) + gkTb2_3*(2*qw*qx + 2*qy*qz) + gkTb3_1*(2*qw*qy + 2*qx*qz) - gkTb3_2*(2*qw*qx - 2*qy*qz) + 1)^(3/2)), (2*gkTb1_1*qw + 2*gkTb2_2*qw + 2*gkTb2_3*qx - 2*gkTb1_3*qy + 4*gkTb1_2*qz - 4*gkTb2_1*qz)/(2*(gkTb1_2*(2*qw*qz + 2*qx*qy) - gkTb2_2*(2*qx^2 + 2*qz^2 - 1) - gkTb1_1*(2*qy^2 + 2*qz^2 - 1) - gkTb3_3*(2*qx^2 + 2*qy^2 - 1) - gkTb1_3*(2*qw*qy - 2*qx*qz) - gkTb2_1*(2*qw*qz - 2*qx*qy) + gkTb2_3*(2*qw*qx + 2*qy*qz) + gkTb3_1*(2*qw*qy + 2*qx*qz) - gkTb3_2*(2*qw*qx - 2*qy*qz) + 1)^(1/2)) - ((gkTb1_2*(2*qx^2 + 2*qz^2 - 1) - gkTb2_1*(2*qy^2 + 2*qz^2 - 1) + gkTb1_1*(2*qw*qz - 2*qx*qy) - gkTb1_3*(2*qw*qx + 2*qy*qz) + gkTb2_2*(2*qw*qz + 2*qx*qy) - gkTb2_3*(2*qw*qy - 2*qx*qz))*(2*gkTb1_2*qw - 2*gkTb2_1*qw + 2*gkTb1_3*qx + 2*gkTb3_1*qx + 2*gkTb2_3*qy + 2*gkTb3_2*qy - 4*gkTb1_1*qz - 4*gkTb2_2*qz))/(4*(gkTb1_2*(2*qw*qz + 2*qx*qy) - gkTb2_2*(2*qx^2 + 2*qz^2 - 1) - gkTb1_1*(2*qy^2 + 2*qz^2 - 1) - gkTb3_3*(2*qx^2 + 2*qy^2 - 1) - gkTb1_3*(2*qw*qy - 2*qx*qz) - gkTb2_1*(2*qw*qz - 2*qx*qy) + gkTb2_3*(2*qw*qx + 2*qy*qz) + gkTb3_1*(2*qw*qy + 2*qx*qz) - gkTb3_2*(2*qw*qx - 2*qy*qz) + 1)^(3/2))];
 

        end 

        function clonedSystem = clone(obj)
            clonedSystem = RobotsObjectSystem(obj.state, obj.sizeState, obj.sizeOutput,obj.SampleTime, obj.Bm,obj.bg,obj.opg1,obj.opg2,obj.oRg1,obj.oRg2,obj.n_pose_measures ...
                                            ,obj.b1pg1,obj.b1Qg1,obj.b2pg2,obj.b2Qg2,obj.b1pb2,obj.b1Qb2,obj.bpb1,obj.bQb1);
        end

    end
end
