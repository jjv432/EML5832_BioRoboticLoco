% Slip Simulation and Animation
% Ryan Kaczmarczyk, Jack Vranicar, and Shane Rober
% 2025-03-24 CE

clc;
clear;
close all;
format compact;

% Add functions in
addpath("src");

% Show Animation?
animate = 1;

% Parameters
params.l0 = 1;  %* m
params.b = 2;  %*
params.k = 100; %*
params.g = 9.81;
params.m = 1;  %*
params.t_hip = .276;  %*
params.phi_0 = -pi/6;  %*
params.phi_d_0 = 1.5;
params.l_d_0 = -3.5;

% Time Vector for ODE45
time = 0:.1:30;

[Kinematics, T] = simulateSystem(params, time);

if animate == 1
    animateHopper(Kinematics, T);
end

figure()
axis equal
plot(Kinematics.X,Kinematics.Z);
xlabel("x-position, m")
ylabel("z-position, m")
title("SLIP-Model Position")