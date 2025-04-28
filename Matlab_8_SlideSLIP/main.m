% Slip Simulation and Animation
% Ryan Kaczmarczyk, Jack Vranicar, and Shane Rober

%{
TODO

Fix the event function for sliding to stance. Should probably just be a >
or < thing for angles instead of subtraction? 


%}
clc;
close all;
clear persistent;
clear functions;
clearvars;

% Add functions in
addpath("src");

params = getParams();

% Time Vector for ODE45
time = 0:.01:30;

params.t_hip = 3;
x_d = 2;

z_d = 0;

z = 1;

x0 =[0; x_d; z; z_d]; % this was our initial

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
    % maxEig = stabilityMeasure(params,time, fixedPoint)
    x0 = fixedPoint;
    writematrix(x0, "FixedPointSaved");
else
    % x0 = readmatrix("FixedPointSaved.txt");
end

%% Simulate System

[Kinematics, T, ~] = simulateSystem(params, time, 0, x0);

%% Animation
% Show Animation?
animate = 1;
if animate == 1
    animateHopper(Kinematics, T);
end

%% Plotting
plotHopper = 1;
if plotHopper == 1
    plotHopperStates(Kinematics, T)
end



