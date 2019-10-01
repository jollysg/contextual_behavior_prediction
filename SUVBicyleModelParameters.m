classdef SUVBicyleModelParameters < VehicleParametersBicycleModel
    properties
        %mass (Kgs)
        m
        %dist of front axle from CG (m)
        l_f
        %dis of rear axle from CG (m)
        l_r
        %Moment of Inertia (Kg-m2)
        Jz
        %Cornering stiffness front axle (Kg/rad) (~ 2 times the tires)
        C_f
        %cornering stiffness rear axle (Kg/rad) (~ 2 times the tires)
        C_r
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