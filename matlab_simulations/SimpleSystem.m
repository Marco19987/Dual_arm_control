% Define a simple system class that implements the necessary methods
classdef SimpleSystem < handle
    properties
        state
        sizeState
        sizeOutput
        sampleTime
    end
    
    methods
        function obj = SimpleSystem(state, sizeState, sizeOutput, sampleTime)
            obj.state = state;
            obj.sizeState = sizeState;
            obj.sizeOutput = sizeOutput;
            obj.sampleTime = sampleTime;
        end
        
        function state = getState(obj)
            state = obj.state;
        end
        
        function sizeState = getSizeState(obj)
            sizeState = obj.sizeState;
        end
        
        function sizeOutput = getSizeOutput(obj)
            sizeOutput = obj.sizeOutput;
        end
        
        function newState = state_fcn(obj, x, u)
            % Define the state transition function
            newState = x + obj.sampleTime*(-x + u);
            obj.state = newState; % update the state of the system
        end
        
        function output = output_fcn(obj, x, u)
            % Define the output function
            output = x(1) + x(2); % Example: direct state as output
        end
        
        function jacobian = jacob_state_fcn(obj, x, u)
            % Define the Jacobian of the state transition function
            jacobian = eye(length(x))-obj.sampleTime*eye(length(x)); % Example: identity matrix
        end
        
        function jacobian = jacob_output_fcn(obj, x, u)
            % Define the Jacobian of the output function
            jacobian = [1 1];%eye(length(x)); % Example: identity matrix
        end
        
        function clonedSystem = clone(obj)
            clonedSystem = SimpleSystem(obj.state, obj.sizeState, obj.sizeOutput,obj.sampleTime);
        end
    end
end

