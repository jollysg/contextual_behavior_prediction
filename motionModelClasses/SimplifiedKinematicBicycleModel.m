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

classdef SimplifiedKinematicBicycleModel < NonLinearMotionModel & MeasurementModel
    properties
        init_states
        vehicleParameters    
    end
    
    methods
        function obj = SimplifiedKinematicBicycleModel(Ts, vp, V)
            if nargin == 0
                % Init to a default vehicle
                V = 10;
                vp = SedanBicycleModelParameters();
                Ts = 0.1;
            elseif nargin == 1
                vp = SedanBicycleModelParameters();
                V = 10;
            elseif nargin == 2
                vp = SedanBicycleModelParameters();
            end
            
            obj.Ts = Ts;

            % states - [x y psi v beta]
            obj.init_states = [0; 0; 0; V; 0];
            obj.states = obj.init_states;            
            obj.vehicleParameters = vp;

            obj.Bd_matrix = [   0 0;
                                0 0;
                                0 0;
                                1 0;
                                0 1] * obj.Ts;

            obj.Cd_matrix = [1 0 0 0 0;
                              0 1 0 0 0];
            
            obj.Dd_matrix = 0;
       end
        
        function F = jacobian(obj, X, u)
            x = X(1);
            y = X(2);
            psi = X(3);
            v = X(4);
            delta = X(5);
            l = obj.vehicleParameters.l_r + obj.vehicleParameters.l_f;
            sin_term = sin(psi);
            cos_term = cos(psi);
            
            F = [   0   0   -v*sin_term     cos_term        0   ; ...
                    0   0   v*cos_term      sin_term        0   ; ...
                    0   0      0            tan(delta)/l   v*(sec(delta)^2)/l; ...
                    0   0      0                0           0   ; ...
                    0   0      0                0           0   ];
        end
        
        function x_apriori = propagate(obj, x, u)
            % input u is [a, omega], 
            
            % [ x y psi v delta]
            % x' = v cos(psi)
            % y' = v sin(psi)
            % psi' = v tan(delta)/l
            % v' = a = u(1)
            % delta' = w = u(2)
            l = obj.vehicleParameters.l_r + obj.vehicleParameters.l_f;
            x_dot = [   x(4) * cos(x(3));
                        x(4) * sin(x(3));
                        x(4) * tan(x(5))/l;
                        u(1);
                        u(2)    ];
            x_apriori = x + x_dot * obj.Ts;
            obj.propagated_states = x_apriori;
        end
        
        % following should return a jacobian in case of non linear motion
        % models
        function F = linearizedDiscreteStateTransitionMatrix(obj, x, u)
            obj.calculateDiscretizedMatrices(x, u);
            F = obj.Fd_matrix;
        end
        
        function calculateDiscretizedMatrices(obj, x, u)
            jacob = obj.jacobian(x, u);
            obj.Fd_matrix = expm(jacob*obj.Ts);
%             trap_term = eye(5) - jacob * obj.Ts/2;
%             obj.Fd_matrix = trap_term\(eye(5) + jacob * obj.Ts/2);
            
            %Also update Bd_matrix, since that has changed too
%             B = [   0 0;
%                     0 0;
%                     0 0;
%                     1 0;
%                     0 1];
% 
%             obj.Bd_matrix = trap_term\B * obj.Ts;
        end
             
    end
end