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
x0 =[0; 1; 1.5; 0]; % this was our initial


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
figure()
hold on
% axis equal
stanceX = Kinematics.X.*(T(:, 2) == 1).*(T(:, 3) == 0);
stanceZ = Kinematics.Z.*(T(:, 2) == 1).*(T(:, 3) == 0);
flightX = Kinematics.X.*(T(:, 2) == 0).*(T(:, 3) == 0);
flightZ = Kinematics.Z.*(T(:, 2) == 0).*(T(:, 3) == 0);
slideX = Kinematics.X.*(T(:, 2) == 0).*(T(:, 3) == 1);
slideZ = Kinematics.Z.*(T(:, 2) == 0).*(T(:, 3) == 1);

stance_tmp = find(stanceX);
flight_tmp = find(flightX);
slide_tmp = find(slideX);

% Flight
idx = 2;
h1 = [];
flight_tmp = [flight_tmp; 0];
while ~isempty(flight_tmp) && idx <= numel(flight_tmp)

    if flight_tmp(idx) - flight_tmp(idx -1) ~=1
        if isempty(h1)
            h1 = plot(flightX(flight_tmp(1:idx-1)), flightZ(flight_tmp(1:idx-1)), 'r', "LineWidth", 2);
        else
            plot(flightX(flight_tmp(1:idx-1)), flightZ(flight_tmp(1:idx-1)), 'r', "LineWidth", 2);
        end
        flight_tmp = flight_tmp(idx:end);
        idx = 2;
    else
        idx = idx + 1;
    end

end

% Slide
idx = 2;
slide_tmp = [slide_tmp; 0];
h2 = [];
while ~isempty(slide_tmp) && idx <= numel(slide_tmp)

    if slide_tmp(idx) - slide_tmp(idx -1) ~=1

        if isempty(h2)
            h2 = plot(slideX(slide_tmp(1:idx-1)), slideZ(slide_tmp(1:idx-1)), 'g', "LineWidth", 2);

        else
            plot(slideX(slide_tmp(1:idx-1)), slideZ(slide_tmp(1:idx-1)), 'g', "LineWidth", 2);

        end
        slide_tmp = slide_tmp(idx:end);
        idx = 2;
    else
        idx = idx + 1;
    end

end

% Stance
idx = 2;
h3 = [];
stance_tmp = [stance_tmp; 0];
while ~isempty(stance_tmp) && idx <= numel(stance_tmp)

    if stance_tmp(idx) - stance_tmp(idx -1) ~=1
        if isempty(h3)
            h3 = plot(stanceX(stance_tmp(1:idx-1)), stanceZ(stance_tmp(1:idx-1)), 'k', "LineWidth", 2);
        else
            plot(stanceX(stance_tmp(1:idx-1)), stanceZ(stance_tmp(1:idx-1)), 'k', "LineWidth", 2);
        end
        stance_tmp = stance_tmp(idx:end);
        idx = 2;
    else
        idx = idx + 1;
    end

end

% yline(max(Kinematics.Z), '--k', "LineWidth", 2);
yline(0, 'k', "LineWidth", 3);
xlabel("x-position, m")
ylabel("z-position, m")
title("SLIP-Model Position")
grid on
legend([h1, h2, h3], {'Flight', 'Sliding', 'Stance'}, 'Location', 'bestoutside');
axis auto
hold off

% figure()
% plot(Kinematics.X, Kinematics.Z)