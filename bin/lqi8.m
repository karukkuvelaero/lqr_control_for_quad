% Define system parameters
g = 9.81;           % acceleration due to gravity
mass_quad = 1.5;    % mass of the quadrotor in kilograms
M_x_quad = 0.0190;  % moment of inertia about x axis in kg*m^2
M_y_quad = 0.0190;  % moment of inertia about y axis in kg*m^2
M_z_quad = 0.0252;  % moment of inertia about z axis in kg*m^2
n_c = 2.021382751247327e-04; % linear relationship between PWM and thrust
n_t = 1.500108339272986;     % linear relationship between PWM and torque
L = 0.25;           % Length of quad arm in metres

% Initial conditions
x = 0;
y = 0;
z = 0;
phi = 0;
theta = 45; % initialize pitch angle value to zero
psi = 0;
u = 0;
v = 0;
w = 0;
p = 0;
q = 0;
r = 0;

% Define system matrices
quad_state_A = [0 0 0 0 0 0 1 0 0 0 0 0 ;
                0 0 0 0 0 0 0 1 0 0 0 0 ; 
                0 0 0 0 0 0 0 0 1 0 0 0 ;
                0 0 0 0 0 0 0 0 0 1 0 0 ;
                0 0 0 0 0 0 0 0 0 0 1 0 ;
                0 0 0 0 0 0 0 0 0 0 0 1 ;
                0 0 0 0 -g 0 0 0 0 0 0 0 ;
                0 0 0 g 0 0 0 0 0 0 0 0 ;
                0 0 0 0 0 0 0 0 0 0 0 0 ;
                0 0 0 0 0 0 0 0 0 0 0 0 ;
                0 0 0 0 0 0 0 0 0 0 0 0 ;
                0 0 0 0 0 0 0 0 0 0 0 0 ];

quad_actuator_B = [zeros(8, 4);
                   [-1/mass_quad, 0, 0, 0;
                    0, 4*L*sin(theta)*n_t/M_x_quad, 0, 0;
                    0, 0, 4*L*cos(theta)*n_t/M_y_quad, 0;
                    0, 0, 0, 4*n_c/M_z_quad]];

observer_C = [0 0 0 0 0 0 0 0 1 0 0 0
              0 0 0 0 0 0 0 0 0 1 0 0
              0 0 0 0 0 0 0 0 0 0 1 0
              0 0 0 0 0 0 0 0 0 0 0 1];


D = zeros(12, 4);
sus = ss(quad_state_A,quad_actuator_B,observer_C,[])

aug_A = [quad_state_A zeros(12,4);-observer_C zeros(4,4)];
aug_B = [quad_actuator_B ;zeros(4,4)]+[zeros(12,4);eye(4,4)];

Ca = [observer_C zeros(4,4); zeros(4,16)];

 ssModel= ss(aug_A,aug_B,Ca,[]);
 f1 = rank(obsv(ssModel))
 f = rank(ctrb(ssModel))
Q = eye(16);

R =  eye (4) ;
N=zeros(16,4);
%l=lqi(sus,Q,R,N)
k = lqr(ssModel,Q,R,N)
