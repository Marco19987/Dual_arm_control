classdef KalmanFilter < handle
    properties
        system
        P
        W
        V
        Identity_x
    end
    
    methods
        function obj = KalmanFilter(system, W, V)
            obj.system = system.clone();
            obj.W = W;
            obj.V = V;
            obj.P = W;
            obj.Identity_x = eye(system.getSizeState());
        end
        
        function obj = KalmanFilterCopy(obj, ss)
            obj.system = ss.system.clone();
            obj.P = ss.P;
            obj.W = ss.W;
            obj.V = ss.V;
            obj.Identity_x = ss.Identity_x;
        end
        
        function setW(obj, W)
            obj.W = W;
        end

        function setP(obj, P)
            obj.P = P;
        end
        
        function setV(obj, V)
            obj.V = V;
        end
        
        function [y_hat_k,x_hat_k_k] = kf_apply(obj, u_k, y_k, W_k, V_k)
            obj.setW(W_k);
            obj.setV(V_k);
            [y_hat_k,x_hat_k_k] = obj.obs_apply(u_k, y_k);
        end
        
        function [y_hat_k,x_hat_k_k] = obs_apply(obj, u_k, y_k)
            x_hat_k1_k1 = obj.system.getState();
            P_k1_k1 = obj.P;
            W_k1 = obj.W;
            V_k = obj.V;
            Identity_x_ = obj.Identity_x;
            
            % PREDICT
            x_hat_k_k1 = obj.system.state_fcn(x_hat_k1_k1, u_k);
            F_k1 = obj.system.jacob_state_fcn(x_hat_k1_k1, u_k);
            P_k_k1 = F_k1 * P_k1_k1 * F_k1' + W_k1;
            
            % UPDATE
            y_hat_k_k1 = obj.system.output_fcn(x_hat_k_k1, u_k);
            y_tilde_k = y_k - y_hat_k_k1;

            H_k = obj.system.jacob_output_fcn(x_hat_k_k1, u_k);
            S_k = H_k * P_k_k1 * H_k' + V_k;
            K_k = P_k_k1 * H_k' / S_k;
            x_hat_k_k = x_hat_k_k1 + K_k * y_tilde_k;
            obj.P = (Identity_x_ - K_k * H_k) * P_k_k1;
            y_hat_k = obj.system.output_fcn(x_hat_k_k, u_k);
        end
    end
end
