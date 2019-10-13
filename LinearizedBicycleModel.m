classdef LinearizedBicycleModel < MotionModel & MeasurementModel
    properties
        % states - [ y y_dot psi psi_dot], input (u) is in radians
        init_states
        vehicleParameters
        a22_c
        a24_c
        a42_c
        a44_c
        bb_2
        bb_4
        V
    end
    
    methods
        function obj = LinearizedBicycleModel(Ts, V, vp)
            if nargin == 0
                % Init to a default vehicle
                V = 10;
                vp = SedanBicycleModelParameters();
                Ts = 0.1;
            elseif nargin == 1
                vp = SedanBicycleModelParameters();
                Ts = 0.1;
            elseif nargin == 2
                vp = SedanBicycleModelParameters();
            end
            
            obj.V = V;
            obj.Ts = Ts;
            
            % use the model from rajamani, with lateral velocity and yaw
            % rate as states - [y vy psi psi_dot]
            obj.init_states = [0; 0; 0; 0];
            obj.states = obj.init_states;
            
            obj.a22_c = -(vp.C_f + vp.C_r)/vp.m;
            obj.a24_c = (vp.C_f*vp.l_f - vp.C_r*vp.l_r)/vp.m;  
            obj.a42_c = - (vp.C_f*vp.l_f - vp.C_r * vp.l_r)/vp.Jz;
            obj.a44_c = - (vp.l_f^2 * vp.C_f + vp.l_r^2 * vp.C_r)/vp.Jz;
            obj.vehicleParameters = vp;
            obj.bb_2 = vp.C_f/vp.m;
            obj.bb_4 = vp.C_f*vp.l_f/vp.Jz;
            
            obj.calculateDiscretizedMatrices(V);
                        
        end
        
        function calculateDiscretizedMatrices(obj, V)
            F = obj.calculateContinuousStateTransitionMatrix(V);
            B = obj.getInputMatrix();
            C = obj.getOutputMatrix();
            D = 0;
            
            % disretize using Numerical integration. Ad = expm(A * Ts), 
            % Bd = (Ad - I)BA^-1
            % the problem is the matrix is non invertible
%             obj.Fd_matrix = expm(F * obj.Ts);
%             
%             
%             obj.Bd_matrix = (obj.Fd_matrix - eye(4))* F\B; 

            % Using euler forward
%             obj.Fd_matrix = (eye(4) + F * obj.Ts);
%             obj.Bd_matrix = B * obj.Ts;
            
            % trapezoidal, works good for linear
            trap_term = eye(4) - F * obj.Ts/2;
            obj.Fd_matrix = trap_term\(eye(4) + F * obj.Ts/2);
            obj.Bd_matrix = trap_term\B * obj.Ts;
            obj.Cd_matrix = C/trap_term;
            obj.Dd_matrix = D + obj.Cd_matrix*B*obj.Ts/2;
            obj.V = V;

        end
        
        function  x_apriori = propagate(obj, x, u)
            % u is in radians
            obj.states = x;
            x_apriori = obj.Fd_matrix * x + obj.Bd_matrix * u;
            obj.propagated_states = x_apriori;
        end
        
        function F = calculateContinuousStateTransitionMatrix(obj, Vx)
            
            % v_dot = -(c1+c2)/(mV^2)v-1/mV(mV+(ac1-bc2)/V)r + c1/(mV) delta
            % r_dot = (-ac1+bc2)/(JzV)v-(a^2C1+b^2C2)/(JzV)r + (ac1/Jz)delta
            % Its important to leave only in terms of V as a variable.
            % i.e., calculate all the other coefficients axx_c beforehand,
            % and then substitute V at runtime.
            a22 = obj.a22_c/Vx;
            a24 = -Vx - obj.a24_c/Vx;
            a42 = obj.a42_c/Vx;
            a44 = obj.a44_c/Vx;
            F = [0 1 0 0; ...
                0 a22 0 a24; ...
                0 0 0 1; ...
                0 a42 0 a44];
        end
        
        function B = getInputMatrix(obj)
            B = [0; obj.bb_2; 0; obj.bb_4];
        end
        
        function C = getOutputMatrix(obj)
            C = [1 0 0 0 ];
        end
        
        function Fd = linearizedDiscreteStateTransitionMatrix(obj, x, u)
            % x not used since the system is linear
            Fd = obj.Fd_matrix;
        end
        
    end
end