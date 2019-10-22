classdef MotionModel < handle %& matlab.mixin.Copyable
    properties
        Ts
        states
        propagated_states
        
        % discretized matrices
        Fd_matrix
        Bd_matrix
        Cd_matrix
        Dd_matrix
    end
    methods (Abstract)
%         x_apriori = propagate(obj, x, u)
        
        % following should return a jacobian in case of non linear motion
        % models
%         F = linearizedDiscreteStateTransitionMatrix(obj, x, u)        
    end
    
    methods
        function  x_apriori = propagate(obj, x, u)
            % u is in radians
            % following should be set only by updates
%             obj.states = x;
            x_apriori = obj.Fd_matrix * x + obj.Bd_matrix * u;
            obj.propagated_states = x_apriori;
        end
        
        function Fd = linearizedDiscreteStateTransitionMatrix(obj, x, u)
            % x not used since the system is linear
            Fd = obj.Fd_matrix;
        end
        
        function c_matrix = getOutputMatrix(self)
            c_matrix = self.Cd_matrix;
        end
        
        function setStates(self, x)
            self.states = x;
        end
    end
end