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
quad_state_A = [zeros(6), eye(6);
                zeros(6), [0, 0, 0, -g, 0, 0;
                           g, 0, 0, 0, 0, 0;
                           zeros(4, 6)]];

quad_actuator_B = [zeros(8, 4);
                   [1/mass_quad, 0, 0, 0;
                    0, 4*L*sin(theta)*n_t/M_x_quad, 0, 0;
                    0, 0, 4*L*cos(theta)*n_t/M_y_quad, 0;
                    0, 0, 0, 4*n_c/M_z_quad]];

observer_C = [zeros(12), eye(12)];

D = zeros(12, 4);

% Define the weighting matrices for LQI controller
Q = diag([1, 1, 0.00001, 0.001, 0.001, 0.001, 1, 1, 0.0001, 0.000001, 0.00001, 0.0001]);
R = 1e-5 * eye(4);

% Augment the Q matrix for integral action
Q_augmented = blkdiag(Q, zeros(12));


% Call the lqi function to compute the controller gains
K_lqi = lqi(quad_state_A, quad_actuator_B, Q, R);

% Extract the state-feedback part
K_feedback = K_lqi(:, 1:12);

% Extract the integral part
K_integral = K_lqi(:, 13:end);

% Create the closed-loop state-space model
A_closed_loop = quad_state_A - quad_actuator_B * K_feedback;
B_integral = quad_actuator_B * K_integral;
sys_closed_loop = ss(A_closed_loop, B_integral, observer_C, D);

% Plot the step response of the closed-loop system
step(sys_closed_loop);
