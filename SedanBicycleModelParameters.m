classdef SedanBicycleModelParameters < VehicleParametersBicycleModel
    properties
        % inherits properties from VehicleParametersBicycleModel, any
        % additional come here
    end
    
    methods
        function obj = SedanBicycleModelParameters()
            obj.m = 1573;
            obj.l_f = 1.1;
            obj.l_r = 1.58;
            obj.Jz = 2873;
            c_tire_front = 80000; % N/rad per tire
            c_tire_rear = 80000;    %N / rad per tire
            obj.C_f = c_tire_front*2;    %160000 N/rad 
            obj.C_r = c_tire_rear*2;     %160000 N/rad
       end
    end

end