classdef SmallSUVBicycleModelParameters < VehicleParametersBicycleModel
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