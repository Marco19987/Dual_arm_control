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
        function obj = RobotsObjectSystem(state, sizeState, sizeOutput,Bm,bg,opg1,opg2,oRg1,oRg2,n_pose_measures ...
                                            ,b1pg1, b1Qg1,b2pg2,b2Qg2,b1pb2,b1Qb2)
            % Call the constructor of the superclass
            obj@SimpleSystem(state, sizeState, sizeOutput);
            
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
            % implementation of xdot = f(x,u)
            bvo = x(8:10);      % object's linear velocity in the base frame
            bomegao = x(11:13); % object's angular velocity in the base frame
            bQo = x(4:7);       % object's quaternion in the base frame
            g1hg1 = u(1:6);     % robot1's grasp wrench in the grasp frame 1
            g2hg2 = u(7:12);    % robot2's grasp wrench in the grasp frame 2
            oh = obj.W * obj.Rbar * [g1hg1;g2hg2];  % resulting wrench in the object frame

            xdot(1:3) = bvo;
            xdot(4:7) = Helper.quaternion_propagation(bQo,bomegao);
            xdot(8:13) = blkdiag(bRo,bRo)*(obj.Bm\oh) + [obj.bg;0;0;0];
        end
        
        function newState = state_fcn(obj, x, u)
            % discretized version
            % xk+1 = xk + SampleTime * xdotk
            newState = x + sampleTime * obj.eval_xdot(x, u);
            obj.state = newState; % update the state of the system
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

           for k = 1:obj.n_pose_measures
                bTo = Helper.transformation_matrix(x(1:3), x(4:7));

                g1To = inv(b1Tg1) * inv(bTb1) * bTo; % measured object pose from robot 1 with the k-th pose estimator in the grasp1 frame
   
                g2To = inv(b2Tg2) * inv(b1Tb2) * inv(bTb1) * bTo; % measured object pose from robot 1 with the k-th pose estimator in the grasp1 frame
                
                output((k-1)*7:1:k*7,1) = [g1To(1:3,4);rotm2quat(g1To(1:3,1:3))]; 
                output(obj.n_pose_measures + 1 + (k-1)*7:1:k*7+obj.n_pose_measures + 1,1) = [g2To(1:3,4);rotm2quat(g2To(1:3,1:3))]; 
           end 
            
        end
        
        function jacobian = jacob_state_fcn(obj, x, u)
            % Override the Jacobian of the state transition function
            jacobian = 2 * eye(length(x)); % Example: different Jacobian
        end
        
        function jacobian = jacob_output_fcn(obj, x, u)
            % Override the Jacobian of the output function
            jacobian = 0.5 * eye(length(x)); % Example: different Jacobian
        end

    end
end
