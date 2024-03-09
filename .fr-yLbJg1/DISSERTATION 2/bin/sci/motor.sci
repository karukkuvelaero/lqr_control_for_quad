PWM = [1000 1300 1350 1400 1450 1500 1550 1600 1650 1700 1750 1800 1850 1900 1950 2000];
Thrust = [0 210 259 309 373 447 536 628 729 814 906 993 1087 1191 1289 1332];
Torque = [0 0.03 0.04 0.05 0.05 0.06 0.08 0.09 0.1 0.11 0.12 0.14 0.15 0.16 0.18 0.18];

// Perform linear regression
[Thrust_slope, Thrust_inter] = reglin(PWM, Thrust);
[Torque_slope, Torque_inter] = reglin(PWM, Torque);

// Plot thrust in one window
figure();
plot(PWM, Thrust, 'b*', PWM, Thrust_slope*PWM + Thrust_inter, 'r-')
xlabel('PWM')
ylabel('Thrust')
title('Thrust vs PWM')
legend(['Thrust data', 'Thrust regression'], 'location', 'northeast')

// Plot torque in another window
figure();

plot(PWM,Torque, 'b*',PWM, Torque_slope*PWM + Torque_inter, 'k-')
xlabel('PWM')
ylabel('Torque')
title('Torque vs PWM')
legend(['Torque data', 'Torque regression'], 'location', 'northeast')
