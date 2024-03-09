g         = 9.81 ; //acceleration due to gravity
mass_quad = 1.5 ; //mass of the quadrotor in kilograms

M_x_quad = 0.0051213871 //moment of inertia about x axis in kg*m^2
M_y_quad = 0.0078197674 //moment of inertia about y axis in kg*m^2
M_z_quad = 0.0056060331 //moment of inertia about z axis in kg*m^2
n_c      = 1.5001083    //linear relationship between PWM and thrust
n_t      =  0.0002021   //linear relationship between PWM and torque
L        = 0.24         //Length of quad arm in metres

x     = 0 ; 
y     = 0 ; 
z     = 0 ;
phi   = 0 ;
theta = 0 ; //initialise pitch angle value to zero
psi   = 0 ;
u     = 0 ;
v     = 0 ;
w     = 0 ;
p     = 0 ;
q     = 0 ; 
r     = 0 ;

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
                0 0 0 0 0 0 0 0 0 0 0 0 ]
//state space model of quadrotor


quad_actuator_B = [0 0 0 0 ;
                   0 0 0 0 ;
                   0 0 0 0 ;
                   0 0 0 0 ;
                   0 0 0 0 ;
                   0 0 0 0 ;
                   0 0 0 0 ;
                   0 0 0 0 ;
                   -1/mass_quad 0 0 0 ;
                   0 4*n_t*L*sin(theta)/M_x_quad 0 0 ;
                   0 0 4*n_t*L*cos(theta)/M_y_quad 0 ;
                   0 0 0 4*n_c/M_z_quad];
                   
Observer_C      = [1 0 0 0 0 0 0 0 0 0 0 0 ;
                   0 1 0 0 0 0 0 0 0 0 0 0 ;
                   0 0 1 0 0 0 0 0 0 0 0 0 ;
                   0 0 0 1 0 0 0 0 0 0 0 0 ;
                   0 0 0 0 1 0 0 0 0 0 0 0 ;
                   0 0 0 0 0 1 0 0 0 0 0 0 ;
                   0 0 0 0 0 0 1 0 0 0 0 0 ;
                   0 0 0 0 0 0 0 1 0 0 0 0 ;
                   0 0 0 0 0 0 0 0 1 0 0 0 ;
                   0 0 0 0 0 0 0 0 0 1 0 0 ;
                   0 0 0 0 0 0 0 0 0 0 1 0 ;
                   0 0 0 0 0 0 0 0 0 0 0 1 ];
                   
                   
 matrix_x      =   [x ; y ; z ; phi ; theta ; psi ; u ; v ; w ; p ;  q ; r];
                   

