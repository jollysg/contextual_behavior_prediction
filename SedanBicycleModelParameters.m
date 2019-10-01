classdef SedanBicycleModelParameters < VehicleParametersBicycleModel
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