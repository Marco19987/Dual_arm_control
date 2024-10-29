classdef cooperative_space_helper
    %COOPERATIVE_SPACE_HELPER Summary of this class goes here
    %   Class to handle the cooperative space for a dual arm system
    
    properties
        R1  % robot 1
        R2  % robot 2
    end
    
    methods
        function obj = cooperative_space_helper(R1,R2)
            %COOPERATIVE_SPACE_HELPER Construct an instance of this class
            %   Pass the robots
            obj.R1 = R1;
            obj.R2 = R2;
        end
        
        function pa = compute_pa(obj,q1,q2)
            %   this method computes the position of the absolute frame wrt
            %   the frame for the given joint positions
            p1 = obj.R1.fkine(q1).t;  
            p2 = obj.R2.fkine(q2).t;
            pa = 0.5*(p1+p2);
        end

        function bRa = compute_Ra(obj,q1,q2)
            %   this method computes the orientation of the absolute frame
            %   wrt the base frame for the given joint positions
            bR1 =  obj.R1.fkine(q1).R;
            bR2 =  obj.R2.fkine(q2).R;
            R12 = bR1'*bR2; % 1R2 ee2 frame wrt ee1 frame 
            [theta12,r12] = tr2angvec(R12);
            R12_half = angvec2r(theta12/2, r12);  
            bRa = bR1*R12_half;
        end
        
        function Rr = compute_Rr(obj,q1,q2)
            % this method computes the relative orientation between end
            % effectors for the given joint positions
            bR1 =  obj.R1.fkine(q1).R;
            bR2 =  obj.R2.fkine(q2).R;
            Rr = bR1'*bR2; % 1R2 ee2 frame wrt ee1 frame 
        end

        function pr = compute_pr(obj,q1,q2)
            %   this method computes the relative position of the robot's end effecotrs
            %   wrt the base frame for the given joint positions
            p1 = obj.R1.fkine(q1).t;  
            p2 = obj.R2.fkine(q2).t;
            pr = (p2-p1);
        end


    end
end

