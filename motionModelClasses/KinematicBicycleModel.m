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

classdef KinematicBicycleModel < NonLinearMotionModel
    properties
         % states are [x, y, psi, v, beta], v is the velocity vector in
%         % inertial frame
        init_states
        % A matrix in case of linear systems, jacobian matrix F in case of
        % non linear
        %discretized F matrix (A) matrix
        vehicleParameters    
    end
    
    methods
        function obj = KinematicBicycleModel(Ts, vp, V)
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
            beta = X(5);
            l_r = obj.vehicleParameters.l_r;
            sin_term = sin(psi+beta);
            cos_term = cos(psi+beta);
            
            F = [   0   0   -v*sin_term     cos_term    -v*sin_term ; ...
                    0   0   v*cos_term      sin_term    v*cos_term ; ...
                    0   0      0         sin(beta)/l_r  v*cos(beta)/l_r ; ...
                    0   0      0                0           0   ; ...
                    0   0      0                0           0   ];
        end
        
        function x_apriori = propagate(obj, x, u)
            % input u is [a, delta], delta needs to be converted to slip
            % (beta)
            vp = obj.vehicleParameters;
            delta = u(2);
            input_beta = atan(vp.l_r/(vp.l_r + vp.l_f)*tan(delta));
            
%             Fd = obj.linearizedDiscreteStateTransitionMatrix(x, u);
%             x_apriori = Fd * x + obj.Bd_matrix*[u(1); input_beta];
            % states = [x y psi v beta]
            % x' = v cos(psi + beta)
            % y' = v sin(psi + beta)
            % psi' = v sin(beta)/lr
            % v' = a = u(1)
            % beta' = arctan (lr/(lf+lr) tan(delta))
            x_dot = [   x(4) * cos(x(3)+x(5)); ...
                        x(4) * sin(x(3)+x(5)); ...
                        x(4) * sin(x(5))/vp.l_r; ...
                                u(1)    ; ...
                             input_beta ]; 
            
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
            % Non linear system. with EKF, better to keep C and D matrix
            % unchanged, so euler forward discretization would be
            % preferable.
           obj.Fd_matrix = expm(jacob*obj.Ts);

            % B, D and C remain unchanged
             
%             trap_term = eye(5) - jacob * obj.Ts/2;
%             obj.Fd_matrix = trap_term\(eye(5) + jacob * obj.Ts/2);
%             obj.Bd_matrix = trap_term\B * obj.Ts;
%             obj.Cd_matrix = C/trap_term;
%             obj.Dd_matrix = D + obj.Cd_matrix*sysB*stime/2;

        end
        
        function y_hat = estimatedMeasurement(obj, x_minus)
        end
    end
end