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
params.phi_0 = -pi/8;
%% Simulations Params
% Time Vector for ODE45
time = 0:.01:30;

% Initial State values for flight
x_d = 5;
z_d = 0;
z = 1;

x0 =[0; x_d; z; z_d];

%% Generate Stability Contours

surfacePlotBool = 0;

if surfacePlotBool

    % First, do it for normal slip model
    model_boolean = 0;
    for i = 1
        createSurfacePlots(params, time, x0, model_boolean)
        params.muS = params.muS - .01;
    end
    
    % Next, do it for sliding slip model
    model_boolean = 1;
    for i = 1
        createSurfacePlots(params, time, x0, model_boolean)
        params.muS = params.muS - .01;
    end
end


%% Generating Data for a Single Simulation of the System

model_boolean = 0;
[Kinematics, T, ~] = simulateSystem(params, time, 0, x0, model_boolean);

%% Animation of the System
% Show Animation?
animate = 1;
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