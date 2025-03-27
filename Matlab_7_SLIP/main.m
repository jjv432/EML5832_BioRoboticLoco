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

params = getParams();

% Time Vector for ODE45
time = 0:.1:30;

% apexHeight: 1.5611 if 1, 1.560 if 0
isInit = 0;

[Kinematics, T, apex] = simulateSystem(params, time, isInit);

apex

if animate == 1
    animateHopper(Kinematics, T);
end

figure()
axis equal
plot(Kinematics.X,Kinematics.Z);
xlabel("x-position, m")
ylabel("z-position, m")
title("SLIP-Model Position")