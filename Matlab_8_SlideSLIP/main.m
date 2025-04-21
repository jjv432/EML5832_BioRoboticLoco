% Slip Simulation and Animation
% Ryan Kaczmarczyk, Jack Vranicar, and Shane Rober

clc;
clear all; % need to clear persistent values
close all;
format compact;

% Add functions in
addpath("src");

params = getParams();

% Time Vector for ODE45
time = [0 30];

% l0, l_d, phi_0, phi_d
l_d = -3.5;
phi_d = 3.5;
params.phi_0 = -1.2;
params.muS = 10; 
params.t_hip = 5;

x0 =[params.l0; l_d; params.phi_0; phi_d]; % this was our initial

%% Running stability functions
% Running newton raphson to get a fixed point
% surfacePlotBool = 0;
% 
% if surfacePlotBool
%     params.phi_0 = -pi/4.5;
%     for i = 1
%         createSurfacePlots(params, time, x0)
%         params.phi_0 = params.phi_0 - .02;
%     end
% end

%%
stabilityBool = 0;
if stabilityBool
    fixedPoint = newtonRaphson(params, time, x0);

    % Finding the stability of the fixed point
    maxEig = stabilityMeasure(params,time, fixedPoint)
    x0 = fixedPoint;
    writematrix(x0, "FixedPointSaved");
else
    % x0 = readmatrix("FixedPointSaved.txt");
end

%% Simulate System

[Kinematics, T, ~] = simulateSystem(params, time, 0, x0);

%% Animation
% Show Animation?
animate = 0;
if animate == 1
    animateHopper(Kinematics, T);
end

%% Plotting
plotHopper = 1;
if plotHopper == 1
    plotHopperStates(Kinematics, T)
end



