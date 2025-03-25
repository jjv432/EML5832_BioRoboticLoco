clc;
clear;
close all;
format compact;

params.l0 = 1;
params.b = 0;
params.k = 100;
params.g = 9.81;
params.m = 1;
params.t_hip = 0;
params.phi_0 = -pi/6;

time = 0:.01:30;

stance_options = odeset('Events', @(t, y) stance_event_func(t,y,params));
flight_options = odeset('Events', @(t, y) flight_event_func(t,y,params));


%% ODE45 Call
Kinematics.X = [];
Kinematics.Z = [];
last_end_time = 0;

T = [];
state = 'stance';

init = [params.l0; -4; params.phi_0; 1.75];

for i = 1:3

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

            % Store the T, x, and z positions for plotting
            

            Kinematics.X = [Kinematics.X; x_vals + x_offset];
            Kinematics.Z = [Kinematics.Z; z_vals];
            T = [T; T1 + t_offset];

            % Set 'state' and the init vector for the next state
            init = [x_vals(end); x_d_vals(end); z_vals(end); z_d_vals(end)];
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

            Kinematics.X = [Kinematics.X; x_vals  + x_offset];
            Kinematics.Z = [Kinematics.Z; z_vals];
            T = [T; T1 + t_offset];

            % Set 'state' and the init vector for the next state
            % phi = tan(z_vals(end)/ x_vals(end));
            phi = params.phi_0;

            l0 = params.l0;
            vx = x_d_vals(end);
            vz = z_d_vals(end);
            
            l_d0 = vz * cos(phi) + vx * sin(phi);
            phi_d0 = l0*(vz*sin(phi) + vx*cos(phi));
            init = [l0; l_d0; phi; 1.75];
            state = 'stance';


    end

end


animateHopper(Kinematics, T);

figure()
axis equal
plot(Kinematics.X,Kinematics.Z);
title("Z v X")
%
% figure()
% plot(T, Kinematics.Z);
% title("Z");
% figure()
% plot(T, Kinematics.X);
% title("X");

%% Animation

function animateHopper(Kinematics, T)

    X = Kinematics.X;
    Z = Kinematics.Z;
    figure();
    grid on
    hold on
    yline(0, 'k--');
    plot(0, 0, 'x', 'LineWidth', 3);
    for i = 2:length(X)
        h1 = plot(X(i), Z(i), 'rx', 'LineWidth',5);
        axis([-4 9 -4 9])
        axis equal
        pause(T(i) - T(i-1));

        delete(h1)
    end


end


%% Dynamics Functions
function func = stance_dynamics(t,y, params)

    persistent b m k l0 g t_hip
    if isempty(b)
        b = params.b;
        k = params.k;
        m = params.m;
        l0 = params.l0;
        g = params.g;
        t_hip = params.t_hip;
    end

    l = y(1);
    l_d = y(2);
    phi = y(3);
    phi_d = y(4);

    l_dd = l*phi_d^2 - g*cos(phi) - (k/m)*(l - l0) - (b/m)*l_d;
    phi_dd = (1/l)*(-2*l_d*phi_d) + (1/l)*(g*sin(phi)) + t_hip/(m*l^2);

    func = [l_d; l_dd; phi_d; phi_dd];

end



function func = flight_dynamics(t, y, params)

    persistent g
    if isempty(g)
        g = params.g;
    end

    x = y(1);
    x_d = y(2);
    z = y(3);
    z_d = y(4);

    x_dd = 0;
    z_dd = -g;

    func = [x_d; x_dd; z_d; z_dd];

end


%% Event Funcs

function [position,isterminal,direction] = flight_event_func(t,y, params)

    % Assuming that the leg 'snaps' to l = l0, phi = phi0
    persistent phi0 l0
    if isempty(phi0)
        phi0 = params.phi_0;
        l0 = params.l0;
    end

    z = y(3);

    position = l0*cos(phi0) - z;
    isterminal = 1;  % Halt integration
    direction = 1;   % Zero approached by decreasing values
end

function [position,isterminal,direction] = stance_event_func(t,y, params)

    persistent k l0 b t_hip
    if isempty(k)
        k = params.k;
        l0 = params.l0;
        b = params.b;
        t_hip = params.t_hip;
    end

    l = y(1);
    l_d = y(2);
    phi = y(3);
    phi_d = y(4);

    Fleg = k*(l0 - l) - b*l_d;

    position = Fleg*cos(phi) + (1/l)*t_hip*sin(phi);
    isterminal = 1;
    direction = -1;
end