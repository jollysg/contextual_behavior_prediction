classdef SUVBicyleModelParameters < VehicleParametersBicycleModel
    properties
    end
    
    methods
        function obj = SUVBicyleModelParameters()
            % need realistic values (C_tire too low for an SUV maybe?)
            obj.m = 2100;
            obj.l_f = 1.3;
            obj.l_r = 1.5;
            obj.Jz = 3900;
            obj.C_f = 2*80000; 
            obj.C_r = 2*80000;
        end
    end
end