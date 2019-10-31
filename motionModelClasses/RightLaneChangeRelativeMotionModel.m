classdef RightLaneChangeRelativeMotionModel < LeftLaneChangeRelativeMotionModel
    properties
%         % Maneuver length
%         L
%         % Lane width
%         Wl
%         % Maneuver amplitude
%         man_A
%         % Maneuver frequency calculated from length, w = 2*Pi*1/L = Pi/L
%         man_w
%         
    end
    
    methods
        function self = RightLaneChangeRelativeMotionModel(Ts, maneuver_length, lane_width)
            self@LeftLaneChangeRelativeMotionModel(Ts, maneuver_length, lane_width);
        end
        
        function x_plus = propagate(self, x, u)
            % states are [x, vx, y, vy, x_mid], where x_mid is point of
            % maneuver initiation
            % Right lane change
            % y propagation = A cos(wx - x_mid) - A + y_init;
            delta_x = x(1) - x(5);
            x_plus = [x(1) + x(2)*self.Ts; ...
                            x(2); ...
                         self.man_A * cos(self.man_w*delta_x) - self.man_A + self.current_lane_y; ...
                         -self.man_A*self.man_w*x(2)*sin(self.man_w*delta_x); ...
                            x(5)] + u;
            self.propagated_states = x_plus;

        end
        
%         function F = linearizedDiscreteStateTransitionMatrix(self, x, u)
%             % This can call jacobian function if needed
%             F = self.jacobian(x,u);
%         end
        
        function dfdx = jacobian(self, x, u)
            % TODO: Correct this jacobian, the following is the linear
            % discretized propagation. The jacobian won't have the 1s
            delta_x = x(1) - x(5);
            dfdx = [   1 self.Ts   0   0   0; 
                    0   1       0   0   0; 
                    -self.man_A*self.man_w*sin(self.man_w*delta_x) 0 0 0 self.man_A*self.man_w*sin(self.man_w*delta_x); 
                    -self.man_A*self.man_w^2*x(2)*cos(self.man_w*delta_x) -self.man_A*self.man_w*sin(self.man_w*delta_x) 0 0 self.man_A*self.man_w^2*x(2)*cos(self.man_w*delta_x); 
                    0 0 0 0 1];
       end
    end
end