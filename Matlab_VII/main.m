clc;
clear;
close all;
format compact;

params.l0 = 1;
params.b = 0;
params.k = 500;
params.g = 9.81;
params.m = 10;
params.t_hip = 1000;
% params.t_hip = 50;
% params.t_hip = 0;
time = [0 30];

% l, ldot, phi, phid
phi_0 = -pi/6;
init = [1; 0; phi_0; 0];

stance_options = odeset('Events', @(t, y) stance_event_func(t,y,params));
flight_options = odeset('Events', @(t, y) flight_event_func(t,y,params));


%% ODE45 Call
Kinematics.X = [];
Kinematics.Z = [];
last_end_time = 0;

T = [];
state = 'stance';

for i = 1:4

    switch state

        case 'stance'

            [T1, Y1] = ode45(@(T1, Y1) stance_dynamics(T1, Y1, params), time, init, stance_options);

            Y1(:, 3) = wrapTo2Pi(Y1(:,3));
            % Y1(:, 3) = -sign(Y1(:,3));
            if ~isempty(Kinematics.X)
                x = Y1(end, 1) * -sign(Y1(end, 3))*sin(Y1(end, 3)) + Kinematics.X(end);
            else
                x = Y1(end, 1) * -sign(Y1(end, 3))*sin(Y1(end, 3));
            end
            % x = Y1(end, 1) * sign(Y1(end, 3))*sin(Y1(end, 3));
            z = Y1(end, 1) * cos(Y1(end, 3));

            x_d = Y1(end, 2) * sin(Y1(end, 3)) + Y1(end, 4) * sin(Y1(end, 3)) * Y1(end, 1);
            z_d = Y1(end, 2) * cos(Y1(end, 3)) + Y1(end, 4) * cos(Y1(end, 3)) * Y1(end, 1);

            init = [x; -x_d; z; z_d; Y1(end, 3); Y1(end, 4)];

            T = [T; T1(1:end-1) + last_end_time];
            last_end_time = max(T1) + last_end_time;

            state = 'flight';
            % Y1(1:end-1, 3) = wrapTo2Pi(Y1(1:end-1, 3));
            % x_array = Y1(1:end-1, 1) .* sin(Y1(1:end-1, 3)) .* (-sign(Y1(1:end-1, 3)));

            if ~isempty(Kinematics.X)
                x_array = Y1(1:end-1, 1) .* sin(Y1(1:end-1, 3)) + Kinematics.X(end);
            else
                x_array = Y1(1:end-1, 1) .* sin(Y1(1:end-1, 3));
            end

            % x_array = Y1(1:end-1, 1) .* sin(Y1(1:end-1, 3));
            z_array = Y1(1:end-1, 1) .* cos(Y1(1:end-1, 3));

            Kinematics.X = [Kinematics.X; x_array]; % End of this one is beginning of the next one, values were duplicated before
            Kinematics.Z = [Kinematics.Z; z_array]; % End of this one is beginning of the next one, values were duplicated before
            clear T1 Y1

        case 'flight'

            disp("flight");
            [T1, Y1] = ode45(@(T1, Y1) flight_dynamics(T1, Y1, params), time, init, flight_options);

            % init = [1; 0; Y1(end, 5); Y1(end, 6)];
            init = [1; 0; -pi/6; 0];
            T = [T; T1(1:end-1) + last_end_time];
            last_end_time = max(T1) + last_end_time;

            state = 'stance';
            Kinematics.X = [Kinematics.X; Y1(1:end-1, 1)]; % End of this one is beginning of the next one, values were duplicated before
            Kinematics.Z = [Kinematics.Z; Y1(1:end-1, 3)];
            clear T1 Y1

    end

end


animateHopper(Kinematics);

figure()
axis equal
plot(Kinematics.X,Kinematics.Z);
title("Z v X")

figure()
plot(T, Kinematics.Z);
title("Z");
figure()
plot(T, Kinematics.X);
title("X");

%% Animation

function animateHopper(Kinematics)

    X = Kinematics.X;
    Z = Kinematics.Z;
    figure();
    hold on
    yline(0, 'k--');
    plot(0, 0, 'x', 'LineWidth', 3);
    for i = 1:length(X)
        h1 = plot(X(i), Z(i), 'rx', 'LineWidth',5);
        axis([-4 9 -4 9])
        axis equal
        pause(.1);

        delete(h1)
    end


end


%% Dynamics Functions
function func = stance_dynamics(t,y, params)

    % l = y(1), ldot = y(2), phi = y(3), phid = y(4), x = y(5), x_d = y(6),
    % z = y(7), z_d = y(8)

    persistent k m b g t_hip l0

    if isempty(k)
        k = params.k;
        m = params.m;
        b = params.b;
        g = params.g;
        t_hip = params.t_hip;
        l0 = params.l0;
    end

    l_dd = y(1)*(y(4))^2 - g*cos(y(3)) - (k/m)*(y(1) - l0) - (b/m)*y(2);

    phi_dd = (-2*y(4)*y(2))/(y(1)) + g*sin(y(3))/y(1) + t_hip/(m*(y(1))^2);

    func = [y(2); l_dd; y(4); phi_dd];
end

% l, ldot, phi, phid
function [position,isterminal,direction] = stance_event_func(t,y, params)

    Fleg = params.k*(params.l0 - y(1)) - params.b*y(2);

    y(3) = wrapTo2Pi(y(3));
    y(3) = -sign(y(3)) * y(3);
    % position = y(3);
    position = Fleg * cos(y(3)) + (params.t_hip/(y(1))) * sin(y(3));
    isterminal = 1;  % Halt integration
    direction = -1;   % Zero approached by decreasing values
end



%% Flight

% x = y(1); x_d = y(2); z = y(3); z_d = y(4); phi = y(5); phi_d = y(6);

function [position,isterminal,direction] = flight_event_func(t,y, params)
    l0 = 1;
    position = y(3) - l0*cos(y(5)); % The value that we want to be zero
    isterminal = 1;  % Halt integration
    direction = -1;   % Zero approached by decreasing values
end



function func = flight_dynamics(t, y, params)

    g = 9.81;
    x_dd = 0;
    z_dd = -g;

    func = [y(2); x_dd; y(4); z_dd; 0; 0];
end


