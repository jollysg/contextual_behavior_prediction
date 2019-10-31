classdef HighDVehicleInfo < VehicleInfo
    properties
        frames
        frame_rate
        init_frame
        final_frame
        trajectories_xvel
        trajectories_yvel
        trajectories_xacc
        trajectories_yacc
        % 1 = -ve, 2 = +ve
        direction
        
    end
    
    methods
        function obj = HighDVehicleInfo(full_dataset, vehicle_id)
            obj = obj@VehicleInfo();
            obj.vehicleID = vehicle_id;
            obj.vehicle_dataset = full_dataset(vehicle_id);
            v = obj.vehicle_dataset;
            obj.lanesOccupied = unique(v.lane);
            obj.numberOfLanesOccupied = v.numLaneChanges+1;
            obj.frame_rate = 25;
            obj.frames = v.frames;
            obj.init_frame = v.initialFrame;
            obj.final_frame = v.finalFrame;
            % TODO: Figure out how to handle times when running multiple
            % vehicles.
            obj.times_s = (obj.frames-obj.frames(1))/obj.frame_rate;
            obj.trajectories_xvel = v.xVelocity;
            obj.trajectories_yvel = v.yVelocity;
            obj.trajectories_xacc = v.xAcceleration;
            obj.trajectories_yacc = v.yAcceleration;
            obj.direction = v.drivingDirection;
            boundingBoxes = v.bbox;
            centroids = [boundingBoxes(:,1) + boundingBoxes(:,3)/2, ...
                boundingBoxes(:,2) + boundingBoxes(:,4)/2];
            obj.trajectories_x = centroids(:,1);
            obj.trajectories_y = centroids(:,2);
        end
        
%         function calculateXYTrajectories(obj)
%             dt = 1/obj.frame_rate;
%             n = length(obj.trajectories_xacc);
%             obj.trajectories_x = zeros(n, 1);
%             obj.trajectories_y = zeros(n, 1);
%             obj.trajectories_x(1) = 0;
%             obj.trajectories_y(1) = 0;
%             
%             acc_multiplier = 0.5*dt*dt;
%             
%             for i = 2:n
%                 % x(k) = x(k-1) + ut + 1/2at^2;
%                 obj.trajectories_x(i) = obj.trajectories_x(i-1) + obj.trajectories_xvel(i-1) * dt + acc_multiplier* obj.trajectories_xacc(i-1);
%                 obj.trajectories_y(i) = obj.trajectories_y(i-1) + obj.trajectories_yvel(i-1) * dt + acc_multiplier* obj.trajectories_yacc(i-1);
%             end            
%         end
        function calculateXYTrajectories(obj)
            dt = 1/obj.frame_rate;
            n = length(obj.trajectories_xacc);
            obj.trajectories_x = zeros(n, 1);
            obj.trajectories_y = zeros(n, 1);
            obj.trajectories_x(1) = 0;
            obj.trajectories_y(1) = 0;
            
            start = tic;
            
            acc_multiplier = 0.5*dt*dt;

            x_inc = obj.trajectories_xvel *dt + acc_multiplier * obj.trajectories_xacc;
%             x_inc = obj.trajectories_xvel *dt * (-1)^obj.direction + (-1)^obj.direction * acc_multiplier * obj.trajectories_xacc;
            y_inc = obj.trajectories_yvel *dt + acc_multiplier * obj.trajectories_yvel;
            obj.trajectories_x = cumsum([0;x_inc(1:n-1)]);
            obj.trajectories_y = cumsum([0;y_inc(1:n-1)]);
        end
        
        function plotVelYTrajectory(obj)
            plot(obj.times_s, obj.trajectories_yvel);
        end
        
        function plotVelXTrajectory(obj)
            plot(obj.times_s, obj.trajectories_xvel);
        end
        
        function plotAccYTrajectory(obj)
            plot(obj.times_s, obj.trajectories_yacc);
        end
        
        function plotAccXTrajectory(obj)
            plot(obj.times_s, obj.trajectories_xacc);
        end
        
        function plotXTrajectory(obj)
            if (isempty(obj.trajectories_x))
                calculateXYTrajectories(obj);
            end
            %call base class method
            plotXTrajectory@VehicleInfo(obj);
        end
        
        function plotYTrajectory(obj)
            if (isempty(obj.trajectories_y))
                obj.calculateXYTrajectories();
            end
            % call base class method
            plotYTrajectory@VehicleInfo(obj);
        end
        
        function plotXYTrajectory(obj)
            if (isempty(obj.trajectories_y))
                obj.calculateXYTrajectories();
            end
            plot(obj.trajectories_x, obj.trajectories_y);
        end

    end
end
