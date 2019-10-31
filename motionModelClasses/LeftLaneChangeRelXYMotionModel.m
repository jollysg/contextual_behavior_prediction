classdef LeftLaneChangeRelXYMotionModel < LeftLaneChangeRelativeMotionModel
    properties
        % Maneuver length
        L
        % Lane width
        Wl
        % Maneuver amplitude
        man_A
        % Maneuver frequency calculated from length, w = 2*Pi*1/L = Pi/L
        man_w
        % init y
        current_lane_y        
    end
    
    methods
        function self = LeftLaneChangeRelXYMotionModel(Ts, maneuver_length, lane_width)
            self@LeftLaneChangeRelativeMotionModel(Ts, maneuver_length, lane_width)
        end
        
        function x_plus = propagate(self, x, u)
            % states are [x, vx, y, vy, x_mid], where x_mid is point of
            % maneuver initiation
            % y propagation = -A cos(wx - x_mid) + A;
            delta_x = x(1) - x(5);
            x_plus = [x(1) + x(2)*self.Ts; ...
                            x(2); ...
                         -self.man_A * cos(self.man_w*delta_x) + self.man_A + self.current_lane_y; ...
                         self.man_A*self.man_w*x(2)*sin(self.man_w*delta_x); ...
                            x(5)] + u;
            self.propagated_states = x_plus;

        end

        function reset(self, x)
            self.current_lane_y = x(3);
        end

    end
end