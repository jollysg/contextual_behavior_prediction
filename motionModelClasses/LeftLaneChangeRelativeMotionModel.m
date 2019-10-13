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
        end
        
        function x_plus = propagate(self, x, u)
            % states are [x, vx, y, vy, x_mid], where x_mid is point of
            % maneuver initiation
            x_plus = [x(1) + x(2)*self.Ts; ...
                            x(2); ...
                         -self.man_A * cos(self.man_w*(x(1)-x(5))) + self.man_A; ...
                         self.man_A*self.man_w*x(2)*sin(self.man_w*(x(1)-x(5))); ...
                            x(5)] + u;
            self.propagated_states = x_plus;

        end
        
        function F = linearizedDiscreteStateTransitionMatrix(self, x, u)
            % This can call jacobian function if needed
            F = [1 self.Ts 0 0; ...
                    0 1 0 0; ...
                    0 0 1 self.Ts; ...
                    self.man_A*self.man_w^2*x(2)*cos(self.man_w*x(1)-pi) ...
                                -self.man_A*self.man_w*sin(self.man_w*x(1)-pi) 0 0];            
        end
        
        function dfdx = jacobian(self, x, u)
            % TODO: Correct this jacobian, the following is the linear
            % discretized propagation. The jacobian won't have the 1s
            dfdx = [1 self.Ts 0 0; ...
                    0 1 0 0; ...
                    0 0 1 self.Ts; ...
                    self.man_A*self.man_w^2*x(2)*cos(self.man_w*x(1)-pi) ...
                                -self.man_A*self.man_w*sin(self.man_w*x(1)-pi) 0 0];
       end
    end
end