classdef ZeroAccelerationMotionModel < MotionModel & MeasurementModel
    properties
    end
    
    methods
        function self = ZeroAccelerationMotionModel(Ts)
            if nargin == 0
                Ts = 0.1;
            end
            self.Ts = Ts;
            
            % states are [x, vx ax, y, vy, ay];
            self.Fd_matrix = [1 self.Ts self.Ts^2/2 0   0   0;
                              0     1       self.Ts 0   0   0;
                              0     0           0   0   0   0;
                              0     0           0   1   self.Ts     self.Ts^2/2;
                              0     0           0   0       1           self.Ts;
                              0     0           0   0       0               0];
            self.Bd_matrix = [0; 0; 0; 0; 0; 0];
            
            self.Cd_matrix = [  1   0   0   0   0   0;
                                0   0   0   1   0   0];
            self.Dd_matrix = [0; 0];
            %states [x vx ax y vy ay]';
            self.states = [0 0 0 0 0 0]';
            self.output_states = [ 0 0]';

        end        
    end
end