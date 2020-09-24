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

classdef SmallSUVBicycleModelParameters < VehicleParametersBicycleModel
    properties
    end
    
    methods
        function obj = SmallSUVBicycleModelParameters()
            obj.m = 1830;
            obj.l_f = 1.211;
            obj.l_r = 1.459;
            obj.Jz = 3070;
            c_tire = 1830; % kg/deg per tire
            multiplier = 0.165; % 16-17% of tire load per 1 rad of slip angle
            distribution = 0.5; % equal between front and rear
            gravity = 9.81;
            obj.C_f = 2 * c_tire * distribution * multiplier * 180/pi * gravity;    %84859 N/rad/tire = 169720
            obj.C_r = obj.C_f;
       end
    end
end