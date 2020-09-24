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