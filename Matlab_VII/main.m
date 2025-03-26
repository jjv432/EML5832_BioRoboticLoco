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
animate = 0;

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

% Ending Conditions for ODE45
stance_options = odeset('Events', @(t, y) stance_event_func(t,y,params));
flight_options = odeset('Events', @(t, y) flight_event_func(t,y,params));


%% ODE45 Call
Kinematics.X = [];
Kinematics.Z = [];
last_end_time = 0;

T = [];
state = 'stance';

init = [params.l0; params.l_d_0; params.phi_0; params.phi_d_0];

for i = 1:20

    switch state

        case 'stance'

            % Run the stance simulation
            [T1, Y1] = ode45(@(T1, Y1) stance_dynamics(T1, Y1, params), time, init, stance_options);

            % Parse out the results
            l_vals = Y1(:, 1);
            l_d_vals = Y1(:, 2);
            phi_vals = Y1(:, 3);
            phi_d_vals = Y1(:, 4);
            
            % Offset values
            if ~isempty(Kinematics.X)
                x_offset = Kinematics.X(end);
                t_offset = T(end);
            else
                x_offset = 0;
                t_offset = 0;
            end

            % Solve for x and z positions and velocities
            x_vals = l_vals .* sin(phi_vals);
            z_vals = l_vals .* cos(phi_vals);
            x_d_vals = l_vals.*phi_d_vals.*sin(phi_vals) + l_d_vals.*sin(phi_vals);
            z_d_vals = l_vals.*phi_d_vals.*cos(phi_vals) + l_d_vals.*cos(phi_vals);

            if i>1
            x_vals = x_vals - x_vals(1);
            end

            % Store the T, x, and z positions for plotting
            Kinematics.X = [Kinematics.X; x_vals + x_offset];
            Kinematics.Z = [Kinematics.Z; z_vals];
            T = [T; T1 + t_offset, boolean(ones(size(T1)))];

            % Set 'state' and the init vector for the next state
            init = [x_vals(end) + x_offset; x_d_vals(end); z_vals(end); z_d_vals(end)];
            state = 'flight';



        case 'flight'

            [T1, Y1] = ode45(@(T1, Y1) flight_dynamics(T1, Y1, params), time, init, flight_options);

            % Parse out the results
            x_vals = Y1(:, 1);
            x_d_vals = Y1(:, 2);
            z_vals = Y1(:, 3);
            z_d_vals = Y1(:, 4);

            % Store the T, x, and z positions for plotting
            if ~isempty(Kinematics.X)
                x_offset = Kinematics.X(end);
                t_offset = T(end);
            else
                x_offset = 0;
                t_offset = 0;
            end

            Kinematics.X = [Kinematics.X; x_vals];
            Kinematics.Z = [Kinematics.Z; z_vals];
            T = [T; T1 + t_offset, zeros(size(T1))];

            % Set 'state' and the init vector for the next state
            % phi = tan(z_vals(end)/ x_vals(end));
            phi = params.phi_0;

            l0 = params.l0;
            vx = x_d_vals(end);
            vz = z_d_vals(end);
            
            l_d0 = vz * cos(phi) + vx * sin(phi);
            phi_d0 = l0*(vz*sin(phi) + vx*cos(phi));
            init = [l0; l_d0; phi; params.phi_d_0];
            state = 'stance';


    end

end

if animate == 1
animateHopper(Kinematics, T);
end

figure()
axis equal
plot(Kinematics.X,Kinematics.Z);
xlabel("x-position, m")
ylabel("z-position, m")
title("SLIP-Model Position")
%
% figure()
% plot(T, Kinematics.Z);
% title("Z");
% figure()
% plot(T, Kinematics.X);
% title("X");


