clc; clear; close all; format compact


ball_response(5);



%% Function
function ball_response(initial_height)
coeff_restitution = .5;

figure();
hold on
grid on

time = 0:.01:5;
init = [initial_height 0];
last_end_time = 0;

% Calculate the response for multiple bounces
for i = 1:5

    % This stops the solving when the ball is on the ground
    options = odeset('Events', @ball_event_func);

    % Calculate the response
    [T1, Y1] = ode45(@dynamics, time, init, options);

    % Plot the response
    plot(T1 + last_end_time, Y1(:, 1), 'b--');
    xlabel("Time (s)")
    ylabel("Ball Height (m)")
    title("Ball Height vs Time")

    % Set the new initial value for the next solving.  Algebraically, the
    % velocity is determined.  Last_end_time is used for plotting the
    % bounces sequentially
    init = [0, -Y1(end, 2)*coeff_restitution];
    last_end_time = max(T1) + last_end_time;

end

    % This function is used to determine when an 'event' occurs; i.e. when
    % the ball hits the ground
    function [position,isterminal,direction] = ball_event_func(t,y)
        position = y(1); % The value that we want to be zero
        isterminal = 1;  % Halt integration
        direction = 0;   % The zero can be approached from either direction
    end

    % Dynamics of the system using the force of gravity as the only force
    function func = dynamics(t, y)

        g = -9.81;

        y1 = y(1);
        y2 = y(2);

        y1p = y(2);
        y2p = g;

        func = [y1p; y2p];
    end

end