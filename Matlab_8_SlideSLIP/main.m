% Slip Simulation and Animation
% Ryan Kaczmarczyk, Jack Vranicar, and Shane Rober

%{
TODO

Fix the event function for sliding to stance. Should probably just be a >
or < thing for angles instead of subtraction? 


%}
%% Configurations
clc;
close all;
clear persistent;
clear functions;
clearvars;

% Add functions in
addpath("src");

params = getParams();

%% Simulations Params
% Time Vector for ODE45
time = 0:.01:30;

% Initial State values for flight
x_d = 7;
z_d = 0;
z = 1;

x0 =[0; x_d; z; z_d];

%% Generate Stability Contours

surfacePlotBool = 1;

if surfacePlotBool

    % First, do it for normal slip model
    for i = 1
        createSurfacePlots(params, time, x0)
        params.muS = params.muS - .01;
    end
    
    % Next, do it for sliding slip model
    for i = 1
        createSurfacePlots(params, time, x0)
        params.muS = params.muS - .01;
    end
end


%% Generating Data for a Single Simulation of the System

[Kinematics, T, ~] = simulateSystem(params, time, 0, x0);

%% Animation of the System
% Show Animation?
animate = 0;
if animate == 1
    animateHopper(Kinematics, T);
end

%% Plotting the system
plotHopper = 0;
if plotHopper == 1
    plotHopperStates(Kinematics, T)
end


%% OLD
% stabilityBool = 0;
% if stabilityBool
%     fixedPoint = newtonRaphson(params, time, x0);
% 
%     % Finding the stability of the fixed point
%     % maxEig = stabilityMeasure(params,time, fixedPoint)
%     x0 = fixedPoint;
%     writematrix(x0, "FixedPointSaved");
% else
%     % x0 = readmatrix("FixedPointSaved.txt");
% end