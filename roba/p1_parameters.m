% Description	Parameter	Value	Units for vehicle P1
vehicle.mass      = 1724;                       %	(kg)       mass
vehicle.Izz	      = 1300;                       %	(kg*m^2)   Yaw Moment of Inertia	
vehicle.Lf        = 1.35;                       %   (m)        COM-Front Axle Distance
vehicle.Lr        = 1.15;                       %   (m)        COM-Rear Axle Distance		
vehicle.L         = vehicle.Lf+vehicle.Lr;      %   (m)        Wheelbase
vehicle.C_a_f     = 75000;                      %   (N/rad)    Front Axle Cornering Stiffness (estimated)	
vehicle.C_a_r     = 135000;                     %   (N/rad)    Rear Axle Cornering Stiffness  (estimated)
vehicle.W         = 1.6256;                     %   (m)        trackwidth
vehicle.R         = 0.3085;                     %   (m)        tire radius

% Roll properties
vehicle.Ix = 800;                               %   (kg*m^2)   roll inertia 
vehicle.h_Gs = 0.39;                            %   (m)        effective roll height, i.e. cg height minus roll center height 
vehicle.b_roll = 4800;                          %   (Nm*s/rad) roll damping
vehicle.k_roll = 160000;                        %   (Nm/rad)   roll stiffness 