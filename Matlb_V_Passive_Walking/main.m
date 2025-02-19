clc; clear; close all; format compact

init = [0.1995; -.199; .3991; -.0156];
time = [0, 4];
% time = [0 4];
[T, Y] = ode45(@(T, Y) swing_stance_leg_dynamics(T, Y), time, init);
figure()
hold on
plot(T, Y(:, 1))
plot(T, Y(:, 3))



%% Swing dynamics

% y= [theta, theta_d, phi, phi_d]

function func = swing_stance_leg_dynamics(t, y)

    g = 9.81;
    g = 1;
    L = 1;
    gamma = 0.009;

    theta = y(1);
    theta_d = y(2);
    phi = y(3);
    phi_d = y(4);

    theta_dd = (g/L)*sin(theta - gamma);
    phi_dd = theta_dd + (theta_d^2)*sin(phi) - (g/L)*cos(theta-gamma)*sin(phi);

    func = [theta_d; theta_dd; phi_d; phi_dd];

    if (phi - 2*theta) == 0
        tempy3 = y(3);
        tempy4 = y(4);

        y(3) = y(1);
        y(4) = y(2);

        y(1) = tempy3;
        y(2) = tempy4;
    end




end