classdef MildAccelerationZeroLateralVelMotionModel < MotionModel & MeasurementModel
    properties
    end
    
    methods
        function self = MildAccelerationZeroLateralVelMotionModel(Ts)
            if nargin == 0
                Ts = 0.1;
            end
            self.Ts = Ts;
            
            % states are [x, vx ax, y, vy, ay];
            self.Fd_matrix = [1 self.Ts self.Ts^2/2 0   0   0;
                              0     1       self.Ts 0   0   0;
                              0     0           1   0   0   0;
                              0     0           0   1   self.Ts     self.Ts^2/2;
                              0     0           0   0       0           self.Ts;
                              0     0           0   0       0               0];
            self.Bd_matrix = [0; 0; 1; 0; 0; 0] * self.Ts;
            
            self.Cd_matrix = [  1   0   0   0   0   0;
                                0   0   0   1   0   0];
            self.Dd_matrix = [0; 0];
            %states [x vx ax y vy ay]';
            self.states = [0 0 0 0 0 0]';
            self.output_states = [ 0 0]';

        end      
        
        function  x_apriori = propagate(obj, x, u)
            % u is in radians
            % following should be set only by updates
%             obj.states = x;
            % 1m/s acceleration
            x_apriori = obj.Fd_matrix * x + obj.Bd_matrix * 1;
            obj.propagated_states = x_apriori;
        end
    end
end