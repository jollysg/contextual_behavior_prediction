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