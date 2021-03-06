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

classdef LeftLaneChangeRelativeMotionModelWithAcc < NonLinearMotionModel & MeasurementModel
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
        function self = LeftLaneChangeRelativeMotionModelWithAcc(Ts, maneuver_length, lane_width)
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

            self.Bd_matrix = [0; 0; 1; 0; 0; 0; 0];
            self.Cd_matrix = [1 0 0 0 0 0 0; 0 0 0 1 0 0 0];
            self.Dd_matrix = [0;0];
            
            %states [x vx ax y vy ay rel_x]';
            self.states = [0 0 0 0 0 0 0]';
            self.output_states = [ 0 0]';
            self.current_lane_y = 0;
        end

        function x_plus = propagate(self, x, u)
            % states are [x, vx, ax, y, vy, ay, x_mid], where x_mid is point of
            % maneuver initiation
            % y propagation = -A cos(wx - x_mid) + A;
            delta_x = x(1) - x(7);
            x_plus = [x(1) + x(2)*self.Ts + x(3)*self.Ts^2/2; ...
                            x(2) + self.Ts * x(3); ...
                                x(3); ...
                         -self.man_A * cos(self.man_w*delta_x) + self.man_A + self.current_lane_y; ...
                         self.man_A*self.man_w*x(2)*sin(self.man_w*delta_x); ...
                         x(3) * self.man_A * self.man_w *sin(self.man_w*delta_x) + self.man_A*self.man_w^2*x(2)^2*cos(self.man_w*delta_x);
                            x(7)] + self.Bd_matrix * u;
            self.propagated_states = x_plus;
        end
        
        function F = linearizedDiscreteStateTransitionMatrix(self, x, u)
            % This can call jacobian function if needed
            F = self.jacobian(x,u);
        end
        
        function dfdx = jacobian(self, x, u)
            % TODO: Correct this jacobian, the following is the linear
            % discretized propagation. The jacobian won't have the 1s
            delta_x = x(1) - x(7);
            awsinwx = self.man_A * self.man_w * sin(self.man_w*delta_x);
            awcoswx = self.man_A * self.man_w * cos(self.man_w*delta_x);
            aw2coswx = awcoswx * self.man_w;
            aw3vsinwx = awsinwx * self.man_w^2 * x(2);
            dfdx = [   1    self.Ts   self.Ts^2/2   0   0   0   0; 
                       0      1      self.Ts        0   0   0   0; 
                       0      0         1           0   0   0   0;
                    awsinwx   0         0           0   0   0   -awsinwx; 
          awcoswx*self.man_w*x(2) awsinwx 0         0   0   0   -self.man_A*self.man_w^2*x(2)*cos(self.man_w*delta_x);
          x(3)*aw2coswx-aw3vsinwx*x(2) aw2coswx 2*aw3vsinwx 0 0 0 -x(3)*aw2coswx+aw3vsinwx*x(2);
                       0      0         0           0   0   0   1];
        end
       
        function reset(self, x)
            % assign the new y position
            self.current_lane_y = x(4);
        end
    end
end