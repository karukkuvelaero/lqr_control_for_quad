m = 2; // [kg]
c = 1; // [Ns/m]
k = 2; // [N/m]
x0 = 0; // [m]
v0 = 0; // [m/s]

A = [0 1;-k/m -c/m];
B = [0;1/m];
C = [1 0];
D = 0;
X0 = [x0 v0];
