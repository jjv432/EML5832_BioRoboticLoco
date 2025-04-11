% Slip Simulation and Animation
% Ryan Kaczmarczyk, Jack Vranicar, and Shane Rober

clc;
clear all; % need to clear persistent values
close all;
format compact;

% Add functions in
addpath("src");

% Show Animation?
animate = 0;

params = getParams();

% Time Vector for ODE45
% time = 0:.01:30;
time = [0 30];

% x0 =[params.l0; params.l_d_0; params.phi_0; params.phi_d_0]; % this was our initial

% x, x_d, z, z_d
x0 =[0; 1; 1.1; 0]; % this was our initial
% x0 =[0; 0; 3; 0]; % this was our initial
% params.phi_0 = pi/50;
% params.t_hip = 0;

%% Running stability functions
% Running newton raphson to get a fixed point
stabilityBool = 1;
if stabilityBool
    fixedPoint = newtonRaphson(params, time, x0);

    % Finding the stability of the fixed point
    maxEig = stabilityMeasure(params,time, fixedPoint)
    x0 = fixedPoint;
end

%% Want to do newton-raphson?
nr_bool = 0;

[Kinematics, T, ~] = simulateSystem(params, time, nr_bool, x0);


%% Animation
if animate == 1
    animateHopper(Kinematics, T);
end

%% Plotting
plotHopperStates(Kinematics, T)
% 
% figure()
% plot(Kinematics.X, Kinematics.Z)
