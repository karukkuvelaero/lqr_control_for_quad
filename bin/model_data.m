g = 9.81; % acceleration due to gravity
mass_quad = 1.5; % mass of the quadrotor in kilograms

M_x_quad = 0.0190;% moment of inertia about x axis in kg*m^2
M_y_quad = 0.0190; % moment of inertia about y axis in kg*m^2
M_z_quad = 0.0252;  % moment of inertia about z axis in kg*m^2
 n_c = 2.021382751247327e-04; % linear relationship between PWM and thrust
n_t = 1.500108339272986; % linear relationship between PWM and torque
L = 0.25; % Length of quad arm in metres

x = 0;
y = 0;
z = 0;
phi = 0;
theta = 45; % initialise pitch angle value to zero
psi = 0;
u = 0;
v = 0;
w = 0;
p = 0;
q = 0;
r = 0;

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
                0 0 0 0 0 0 0 0 0 0 0 0 ]; % state space model of quadrotor

quad_actuator_B = [0 0 0 0 ;
                   0 0 0 0 ;
                   0 0 0 0 ;
                   0 0 0 0 ;
                   0 0 0 0 ;
                   0 0 0 0 ;
                   0 0 0 0 ;
                   0 0 0 0 ;
                   1/mass_quad 0 0 0 ;
                   0 4*L*sin(theta)*n_t/M_x_quad 0 0 ;
                   0 0 4*L*cos(theta)*n_t/M_y_quad 0 ;
                   0 0 0 4*n_c/M_z_quad];

observer_C = [0 0 0 0 0 0 0 0 0 0 0 0 ;
              0 0 0 0 0 0 0 0 0 0 0 0 ;
              0 0 0 0 0 0 0 0 0 0 0 0 ;
              0 0 0 0 0 0 0 0 0 0 0 0 ;
              0 0 0 0 0 0 0 0 0 0 0 0 ;
              0 0 0 0 0 0 0 0 0 0 0 0 ;
              0 0 0 0 0 0 0 0 0 0 0 0 ;
              0 0 0 0 0 0 0 0 0 0 0 0 ;
              0 0 0 0 0 0 0 0 1 0 0 0 ;
              0 0 0 0 0 0 0 0 0 1 0 0 ;
              0 0 0 0 0 0 0 0 0 0 1 0 ;
              0 0 0 0 0 0 0 0 0 0 0 1 ];

% Aa = [quad_state_A [0;0;0;0;0;0;0;0;0;0;0;0]
%                     [0;0;0;0;0;0;0;0;0;0;0;0]
%                     [0;0;0;0;0;0;0;0;0;0;0;0]
%                     [0;0;0;0;0;0;0;0;0;0;0;0]
%                     [0;0;0;0;0;0;0;0;0;0;0;0]
%                     [0;0;0;0;0;0;0;0;0;0;0;0]
%                     [0;0;0;0;0;0;0;0;0;0;0;0]
%                     [0;0;0;0;0;0;0;0;0;0;0;0]
%                     [0;0;0;0;0;0;0;0;0;0;0;0]
%                     [0;0;0;0;0;0;0;0;0;0;0;0]
%                     [0;0;0;0;0;0;0;0;0;0;0;0]
%                     [0;0;0;0;0;0;0;0;0;0;0;0];
%                     -observer_C [0;0;0;0;0;0;0;0;0;0;0;0;0]
%                     [0;0;0;0;0;0;0;0;0;0;0;0]
%                     [0;0;0;0;0;0;0;0;0;0;0;0]
%                     [0;0;0;0;0;0;0;0;0;0;0;0]
%                     [0;0;0;0;0;0;0;0;0;0;0;0]
%                     [0;0;0;0;0;0;0;0;0;0;0;0]
%                     [0;0;0;0;0;0;0;0;0;0;0;0]
%                     [0;0;0;0;0;0;0;0;0;0;0;0]
%                     [0;0;0;0;0;0;0;0;0;0;0;0]
%                     [0;0;0;0;0;0;0;0;0;0;0;0]
%                     [0;0;0;0;0;0;0;0;0;0;0;0]
%                     [0;0;0;0;0;0;0;0;0;0;0;0];]
% Ba=[quad_actuator_B; [0 0 0 0]]
% Ca=[observer_C,[0;0;0;0;0;0;0;0;0;0;0;0]]
% ssModel = ss (Aa,Ba,Ca,[])
matrix_x = [x; y; z; phi; theta; psi; u; v; w; p; q; r];
D=zeros(12,4);

Q =           [1 0 0 0 0 0 0 0 0 0 0 0 ;
              0 1 0 0 0 0 0 0 0 0 0 0 ;
              0 0 0.00001 0 0 0 0 0 0 0 0 0 ;
              0 0 0 0.001 0 0 0 0 0 0 0 0 ;
              0 0 0 0 0.001 0 0 0 0 0 0 0 ;
              0 0 0 0 0 0.001 0 0 0 0 0 0 ;
              0 0 0 0 0 0 1 0 0 0 0 0 ;
              0 0 0 0 0 0 0 1 0 0 0 0 ;
              0 0 0 0 0 0 0 0 0.0001 0 0 0 ;
              0 0 0 0 0 0 0 0 0 0.000001 0 0 ;
              0 0 0 0 0 0 0 0 0 0 0.00001 0 ;
              0 0 0 0 0 0 0 0 0 0 0 0.0001];
zero_12 = zeros(12)

Q1=[Q [zero_12];zero_12 zero_12]

R = [10e-6 0 0 0;
    0 10e-6 0 0;
    0 0 10e-6 0;
    0 0 0 10e-6];
%K=lqr(quad_state_A,quad_actuator_B,Q,R)
n=ctrb(quad_state_A,quad_actuator_B)
rank(n)
sys=ss(quad_state_A,quad_actuator_B,observer_C,D)
step(sys)
Q2=eye(24)
R2=eye(4)
K2=lqi(quad_state_A,quad_actuator_B,Q1,R2)
%Y=feedback(sys,K,-1)
% step(Y)

