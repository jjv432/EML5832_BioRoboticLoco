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
time = 0:.01:30;

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


        case 'flight'

            [T1, Y1] = ode45(@(T1, Y1) flight_dynamics(T1, Y1, params), time, init, flight_options);


    end

end


animateHopper(Kinematics, T);

% figure()
% axis equal
% plot(Kinematics.X,Kinematics.Z);
% title("Z v X")
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

    persistent l0 b k g m
    if isempty(l0)
        l0 = params.l0;
        b = params.b;
        k = params.k;
        g = params.g;
        m = params.m;
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

    x_dd = 0;
    z_dd = -g;

    func = [y(2); x_dd; y(4); z_dd];
end


%% Event Functions
% l, ldot, phi, phid
function [position,isterminal,direction] = stance_event_func(t,y, params)

    persistent k l0 b
    if isempty(k)
        k = params.k;
        l0 = params.l0;
        b = params.b;
    end

    l = y(1);
    l_d = y(2);
    phi = y(3);
    phi_d = y(4);

    Fleg = k*(l0 - l) - b*l;

    position = Fleg * cos(phi) + (1/l)*(t_hip*sin(phi));
    isterminal = 1;  % Halt integration
    direction = -1;   % Zero approached by decreasing values
end

function [position,isterminal,direction] = flight_event_func(t,y, params)

    persistent l0
    if isempty(l0)
        l0 = params.l0;
    end
    
    position = y(3) - l0*cos(y(5)); % The value that we want to be zero
    isterminal = 1;  % Halt integration
    direction = -1;   % Zero approached by decreasing values
end


