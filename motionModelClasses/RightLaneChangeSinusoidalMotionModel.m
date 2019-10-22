classdef RightLaneChangeSinusoidalMotionModel < LeftLaneChangeSinusoidalMotionModel
    methods
        function self = RightLaneChangeSinusoidalMotionModel(Ts, maneuver_length, lane_width)
            self@LeftLaneChangeSinusoidalMotionModel(Ts, maneuver_length, lane_width);
%             self.Ts = Ts;
%             self.L = maneuver_length;
%             self.Wl = lane_width;
%             self.man_A = lane_width/2;
%             self.man_w = pi/maneuver_length;
%             self.Cd_matrix = [1 0 0 0; 0 0 1 0];
%             self.Dd_matrix = [0;0];
        end
        
        function x_plus = propagate(self, x, u)
            % states are [x, vx, y, vy]
            % x propagation: linear constant velocity
            % y propagation: y = A Cos (wx) -A, v = -Awv Sin(wx)
            x_plus = [x(1) + x(2)*self.Ts; ...
                            x(2); ...
                       x(3) + x(4)*self.Ts; ...
                       -self.man_A*self.man_w*x(2)*sin(self.man_w*x(1))] + u;
            self.propagated_states = x_plus;
        end
        
        % following is called from linearizedDiscreteStateTransitionMatrix
        % from the base class
        function dfdx = jacobian(self, x, u)
            % TODO: Correct this jacobian, the following is the linear
            % discretized propagation. The jacobian won't have the 1s
            dfdx = [1 self.Ts 0 0; ...
                    0 1 0 0; ...
                    0 0 1 self.Ts; ...
                    self.man_A*self.man_w^2*x(2)*cos(self.man_w*x(1)) ...
                        -self.man_A*self.man_w*sin(self.man_w*x(1)) 0 0];
       end
    end
end