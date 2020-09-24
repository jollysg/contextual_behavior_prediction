% MIT License
%
% Copyright (c) 2020 Jasprit Singh Gill (jaspritsgill@gmail.com)
%
% Permission is hereby granted, free of charge, to any person obtaining a copy
% of this software and associated documentation files (the "Software"), to deal
% in the Software without restriction, including without limitation the rights
% to use, copy, modify, merge, publish, distribute, sublicense, and/or sell
% copies of the Software, and to permit persons to whom the Software is
% furnished to do so, subject to the following conditions:

% The above copyright notice and this permission notice shall be included in all
% copies or substantial portions of the Software.

% THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
% IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
% FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
% AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
% LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
% OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN THE
% SOFTWARE.

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
        
        function reset(self, x)
        end
    end
end