% Slip Simulation and Animation
% Ryan Kaczmarczyk, Jack Vranicar, and Shane Rober

clc;
clear all; % need to clear persistent values
close all;
format compact;

% Add functions in
addpath("src");

% Show Animation?
animate = 1;

params = getParams();

% Time Vector for ODE45
time = 0:.01:30;

x0 =[params.l0; params.l_d_0; params.phi_0; params.phi_d_0]; % this was our initial 

nr_bool = 0;

[Kinematics, T, ~] = simulateSystem(params, time, nr_bool, x0);


%% Animation
if animate == 1
    animateHopper(Kinematics, T);
end

%% Plotting
figure()
hold on
% axis equal
plot(Kinematics.X,Kinematics.Z, "LineWidth", 2);
yline(max(Kinematics.Z), '--k', "LineWidth", 2);
yline(0, 'k', "LineWidth", 3);
xlabel("x-position, m")
ylabel("z-position, m")
title("SLIP-Model Position")
grid on
legend("Trajectory", "Max Height", "Ground", "Location", "bestoutside");
ylim([-.1 1.8]);