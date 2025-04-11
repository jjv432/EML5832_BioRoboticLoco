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
time = [0 30];

% x, x_d, z, z_d
x0 =[0; 1; 1.1; 0]; % this was our initial


%% Running stability functions
% Running newton raphson to get a fixed point
params.phi_0 = -pi/4.5;
for i = 1:4

    createSurfacePlots(params, time, x0)
    params.phi_0 = params.phi_0 - .02;
end

%%
stabilityBool = 1;
if stabilityBool
    fixedPoint = newtonRaphson(params, time, x0);

    % Finding the stability of the fixed point
    maxEig = stabilityMeasure(params,time, fixedPoint);
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
plotHopper = 0;
if plotHopper == 1
    plotHopperStates(Kinematics, T)
end