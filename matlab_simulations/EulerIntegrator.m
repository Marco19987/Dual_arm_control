classdef EulerIntegrator < DiscreteSystem
    properties
        continuous_system
    end
    
    methods
        function obj = EulerIntegrator(SampleTime,continuous_system)
            obj@DiscreteSystem(continuous_system.getState(), continuous_system.sizeState, continuous_system.sizeOutput,SampleTime);
            obj.continuous_system = continuous_system;
        end
        
        function state = getState(obj)
            state = obj.state;
        end
        
        function sizeState = getSizeState(obj)
            sizeState = obj.sizeState;
        end

        function updateState(obj,state)
            obj.state = state;
        end
        
        function sizeOutput = getSizeOutput(obj)
            sizeOutput = obj.sizeOutput;
        end
        
        function newState = state_fcn(obj, x, u)
            % Define the state transition function xk+1 = xk + dt*f(xk,uk);
            newState = x + obj.SampleTime*obj.continuous_system.state_fcn(x,u);
        end
        
        function output = output_fcn(obj, x, u)
            % Define the output function
            output = obj.continuous_system.output_fcn(x,u);
        end
        
        function jacobian = jacob_state_fcn(obj, x, u)
            % Define the Jacobian of the state transition function
            jacobian = eye(length(x)) + obj.SampleTime*obj.continuous_system.jacob_state_fcn(x,u); % Example: identity matrix
        end
        
        function jacobian = jacob_output_fcn(obj, x, u)
            % Define the Jacobian of the output function
            jacobian = obj.continuous_system.jacob_output_fcn(x,u);
        end
        
        function clonedSystem = clone(obj)
            clonedSystem = EulerIntegrator(obj.SampleTime,obj.continuous_system);
        end
    end
end

