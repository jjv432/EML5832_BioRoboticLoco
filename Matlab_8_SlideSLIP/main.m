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
params.t_hip = 2;
createSurfacePlots(params, time, x0)

params.t_hip = 7;
createSurfacePlots(params, time, x0)

params.t_hip = 15;
createSurfacePlots(params, time, x0)


function createSurfacePlots(params, time, x0)
    % x-axis is the initial z value, z_d value is the y-axis, stability is the
    % vertical
    persistent plotCount
    if isempty(plotCount)
        plotCount = 1;
    end
    fixedPoint = newtonRaphson(params, time, x0);
    fixedPointStability = stabilityMeasure(params,time, fixedPoint);
    fixedPointZ = fixedPoint(2);
    fixedPointZ_D = fixedPoint(3);

    % setting up plot parameters
    max_z_val = ceil(2*fixedPointZ);
    max_z_d_val = ceil(2*fixedPointZ_D);

    sampleResolution = .1;
    z_sample_points = 0:sampleResolution:max_z_val;
    z_d_sample_points = 0:sampleResolution:max_z_d_val;
    stabilities = zeros(numel(z_sample_points), numel(z_d_sample_points));

    for i = 1:numel(z_sample_points)
        for j = 1:numel(z_d_sample_points)

            curZ = z_sample_points(i);
            curZ_D = z_d_sample_points(j);
            samplePoint = [0; 1; curZ; curZ_D];
            stabilities(i, j) = stabilityMeasure(params,time, samplePoint);

        end
    end

    figure()
    ax = gca;
    hold on
    [X, Y] = meshgrid(z_d_sample_points, z_sample_points);
    contourf(X, Y, real(stabilities));
    ylabel("Z Vals");
    xlabel("Z_D Vals");
    colorbar(ax,"eastoutside");
    title("Stability for " + num2str(params.t_hip));
    saveas(gcf, "Stability" + plotCount + ".png");

    plotCount = plotCount + 1;

end

%%
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
plotHopper = 0;
if plotHopper == 1
    plotHopperStates(Kinematics, T)
end
%
% figure()
% plot(Kinematics.X, Kinematics.Z)
