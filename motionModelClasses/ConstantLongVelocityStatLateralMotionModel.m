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

classdef ConstantLongVelocityStatLateralMotionModel < MotionModel & MeasurementModel
    properties
    end
    
    methods
        function self = ConstantLongVelocityStatLateralMotionModel(Ts)
            if nargin == 0
                Ts = 0.1;
            end
            self.Ts = Ts;
            self.Fd_matrix = [1 self.Ts 0 0;
                              0     1   0 0;
                              0     0   1   self.Ts;
                              0     0   0       0];
            self.Bd_matrix = [0; 0; 0; 0];
            
            self.Cd_matrix = [  1   0   0   0;
                                0   0   1   0];
            self.Dd_matrix = [0; 0];
            %states [x vx y vy]';
            self.states = [0 0 0 0]';
            self.output_states = [ 0 0]';
        end        
    end
end