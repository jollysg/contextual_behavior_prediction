classdef LeftLaneChangeRelativeMotionModel < NonLinearMotionModel & MeasurementModel
    properties
        % Maneuver length
        L
        % Lane width
        Wl
        % Maneuver amplitude
        man_A
        % Maneuver frequency calculated from length, w = 2*Pi*1/L = Pi/L
        man_w
        
    end
    
    methods
        function self = LeftLaneChangeRelativeMotionModel(Ts, maneuver_length, lane_width)
            if nargin == 0
                Ts = 0.1;
                maneuver_length = 10;
                lane_width = 3.5;   
            elseif nargin == 1
                maneuver_length = 10;
                lane_width = 3.5;
            elseif nargin == 2
                lane_width = 3.5;
            end
            
            self.Ts = Ts;
            self.L = maneuver_length;
            self.Wl = lane_width;
            self.man_A = lane_width/2;
            self.man_w = pi/maneuver_length;

            self.Bd_matrix = [0; 0; 0; 0; 0];
            self.Cd_matrix = [1 0 0 0 0; 0 0 1 0 0];
            self.Dd_matrix = [0;0];
            
            %states [x vx y vy rel_x]';
            self.states = [0 0 0 0 0]';
            self.output_states = [ 0 0]';
        end
        
        function x_plus = propagate(self, x, u)
            % states are [x, vx, y, vy, x_mid], where x_mid is point of
            % maneuver initiation
            % y propagation = -A cos(wx - x_mid) + A;
            delta_x = x(1) - x(5);
            x_plus = [x(1) + x(2)*self.Ts; ...
                            x(2); ...
                         -self.man_A * cos(self.man_w*delta_x) + self.man_A; ...
                         self.man_A*self.man_w*x(2)*sin(self.man_w*delta_x); ...
                            x(5)] + u;
            self.propagated_states = x_plus;

        end
        
        function F = linearizedDiscreteStateTransitionMatrix(self, x, u)
            % This can call jacobian function if needed
            F = self.jacobian(x,u);
        end
        
        function dfdx = jacobian(self, x, u)
            % TODO: Correct this jacobian, the following is the linear
            % discretized propagation. The jacobian won't have the 1s
            delta_x = x(1) - x(5);
            dfdx = [   1 self.Ts   0   0   0; 
                    0   1       0   0   0; 
                    self.man_A*self.man_w*sin(self.man_w*delta_x) 0 0 0 -self.man_A*self.man_w*sin(self.man_w*delta_x); 
                    self.man_A*self.man_w^2*x(2)*cos(self.man_w*delta_x) self.man_A*self.man_w*sin(self.man_w*delta_x) 0 0 -self.man_A*self.man_w^2*x(2)*cos(self.man_w*delta_x); 
                    0 0 0 0 1];
       end
    end
end