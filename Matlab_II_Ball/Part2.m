clc; clear; close all; format compact

%{
    In order to model the spring-mass damper effect, I am consider a single
    small mass 'suspsended' inside of the casing of the outside of the
    ball.  This way I am able to use a non-zero height for the dynamics of
    the bounce phase
%}
ball_response(1);


%% Function
function ball_response(initial_height)
state = "flight";

ball_radius = .2;
spring_natrual_length = .02;

figure();
hold on
grid on

time = 0:.01:5;
init = [initial_height 0];
last_end_time = 0;

% This stops the solving when the ball is on the ground
flight_options = odeset('Events', @flight_event_func);
stance_options = odeset('Events', @stance_event_func);

% Calculate the response for multiple bounces
for i = 1:5

    switch(state)

        case "flight"
            % Calculate the response
            [T1, Y1] = ode45(@flight_dynamics, time, init, flight_options);

            % Plot the response
            plot(T1 + last_end_time, Y1(:, 1), 'b--');
            xlabel("Time (s)")
            ylabel("Ball Height at Center (m)")
            title("Ball Height vs Time")

            % Set the new initial value for the next solving.  Algebraically, the
            % velocity is determined.  Last_end_time is used for plotting the
            % bounces sequentially
            init = [Y1(end, 1), Y1(end, 2)];
            last_end_time = max(T1) + last_end_time;
            state = "stance";
            clear T1 Y1

        case "stance"
             % Calculate the response
            [T1, Y1] = ode45(@stance_dynamics, time, init, stance_options);

            % Plot the response
            plot(T1 + last_end_time, Y1(:, 1), 'r--');
            xlabel("Time (s)")
            ylabel("Ball Height at Center (m)")
            title("Ball Height vs Time")

            % Set the new initial value for the next solving.  Algebraically, the
            % velocity is determined.  Last_end_time is used for plotting the
            % bounces sequentially
            init = [Y1(end, 1), Y1(end, 2)];
            last_end_time = max(T1) + last_end_time;
            state = "flight";
            clear T1 Y1


    end


end

% This function is used to determine when an 'event' occurs; i.e. when
% the ball hits the ground
    function [position,isterminal,direction] = flight_event_func(t,y)
        position = y(1) - ball_radius; % The value that we want to be zero
        isterminal = 1;  % Halt integration
        direction = -1;   % Zero approached by decreasing values
    end

    function [position,isterminal,direction] = stance_event_func(t,y)
        position = y(1) - spring_natrual_length; % The value that we want to be zero
        isterminal = 1;  % Halt integration
        direction = 1;   % Zero approached by increasing values
    end

% Dynamics of the system using the force of gravity as the only force
    function func = flight_dynamics(t, y)

        g = -9.81;

        y1 = y(1);
        y2 = y(2);

        y1p = y(2);
        y2p = g;

        func = [y1p; y2p];
    end

    function func = stance_dynamics(t,y)

        g = 9.81;
        kp = 50;
        kd = 1;
        m = .5;
        L = spring_natrual_length;

        y1 = y(1);
        y2 = y(2);

        y1p = y(2);
        y2p = kp*(L - y(1))/m + kd*y1p/m -g;

        func = [y1p; y2p];
    end

end