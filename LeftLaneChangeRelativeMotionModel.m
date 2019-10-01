classdef LeftLaneChangeRelativeMotionModel < MotionModel
    properties
        %sampling time    
        Ts
        % Amplitude of Sinusoidal wave
        A
        % angular frequency of sinusoidal wave
        w
        
        % states = [x vx y vy rel_x]
        states
        
        propagated_states
    end
    
    methods
        function obj = LeftLaneChangeRelativeMotionModel(sample_time, maneuver_length, lane_width)
            
            if nargin == 0
                sample_time = 0.01;
                maneuver_length = 150;  %m
                lane_width = 3.5;
            end
 
            obj.Ts = sample_time;
            obj.A = lane_width/2;
            f = 1/(2*maneuver_length);
            obj.w = 2*pi*f;
            obj.states = [];
        end
        
        function x_apriori = propagate(obj, x, u)
            % Sinusoidal lane change function. Format of states X = [x, vx,
            % y, vy, x_mip], where, x is position, y is position, vx and vy
            % are resp. velocities and x_mip is the maneuver initiation
            % position, i.e. the start of the maneuver. 
            x_apriori = [x(1) + x(2)*obj.Ts; x(2); ...
                -obj.A * cos(obj.w*(x(1)-x(5))) + obj.A; ...
                obj.A* obj.w * x(2)*sin(obj.w *(x(1)-x(5))); x(5)] + u;
            obj.states = x;
            obj.propagated_states = x_apriori;
        end
        
        function dfdx = jacobian_propagation(obj, x, u)
            % Sinusoidal lane change function. Format of states X = [x, vx,
            % y, vy, x_mip], where, x is position, y is position, vx and vy
            % are resp. velocities and x_mip is the maneuver initiation
            % position, i.e. the start of the maneuver.
            A = obj.A;
            w = obj.w;
            dfdx = [1 obj.Ts 0 0 0; ...
                0 1 0 0 0; ...
                A*w*sin(w*(x(1)-x(5))) 0 0 0 -A*w*sin(w*(x(1)-x(5))); ...
                A*w*w*x(2)*cos(w*(x(1)-x(5))) A*w*sin(w*(x(1)-x(5))) 0 0 -A*w^2*x(2)*cos(w*(x(1)-x(5))); ...
                0 0 0 0 1];
        end
        
        function F = linearizedDiscreteStateTransitionMatrix(obj, x, u)
            F = obj.jacobian_propagation(x, u);
        end
    end
    
end