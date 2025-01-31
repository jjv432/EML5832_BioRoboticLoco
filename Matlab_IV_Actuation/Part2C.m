clc; clear; close all; format compact

%{

%}

drop_height = .10;
Kinematics = ball_response(drop_height);
Ball = create_ball(.02);
draw_ball(Ball, Kinematics, drop_height);

%% Functions

%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%%%%                PLOTTING FUNCTIONS                 %%%%
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

function Ball = create_ball(radius)

    thetas = linspace(0, 2*pi, 100);
    x_points = radius * cos(thetas);
    y_points = radius * sin(thetas);
    Ball.X_Points = x_points;
    Ball.Y_Points = y_points;
    Ball.Radius = radius;

end

function draw_ball(Ball, Kinematics, drop_height)
    figure();
    axis equal
    axis([-4*Ball.Radius 4*Ball.Radius -.1 , drop_height*1.2]);
    hold on
    grid on
    title("Spring Mass Damper Response of a Ball");
    xlabel("Horizontal Position (m)")
    ylabel("Vertical Position (m)")
    vid = VideoWriter("MatlabIII_Pt1.avi");
    fps = 60;
    vid.FrameRate = fps;
    vid.Quality = 85;
    % open(vid);

    % for i = 1:numel(Kinematics.Y_Position) %Ensures video is 10 seconds long
    %     cla
    %     fill(Ball.X_Points, Ball.Y_Points + Kinematics.Y_Position(i), 'r');
    %     drawnow;
    %     % writeVideo(vid, getframe(gcf));
    % end

    % close(vid);
    figure(); 
    plot(Kinematics.T, Kinematics.Y_Position)
    title("Time Response of the Ball");
    xlabel("Time (s)");
    ylabel("Vertical Position (m)");
end


%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%%%%              STATE-BASED FUNCTION                 %%%%
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

function Kinematics = ball_response(initial_height)

    Kinematics.Y_Position = [];

    state = "flight";

    time = [0, 100];
    init = [initial_height 0];
    last_end_time = 0;

    % This stops the solving when the ball is on the ground
    flight_options = odeset('Events', @flight_event_func);
    no_motor_options = odeset('Events', @stance_no_motor_event_func);
    motor_options = odeset('Events', @stance_motor_event_func);
    
    T = [];

    % Calculate the response for multiple bounces
    for i = 1:15

        switch(state)

            case "flight"
                % Calculate the response
                [T1, Y1] = ode45(@flight_dynamics, time, init, flight_options);

                % Set the new initial value for the next solving.  Algebraically, the
                % velocity is determined.  Last_end_time is used for plotting the
                % bounces sequentially
                init = [Y1(end, 1), Y1(end, 2)];
                T = [T; T1(1:end-1) + last_end_time];
                last_end_time = max(T1) + last_end_time;

                state = "stance_no_motor";
                Kinematics.Y_Position = [Kinematics.Y_Position; Y1(1:end-1, 1)]; % End of this one is beginning of the next one, values were duplicated before
                clear T1 Y1

            case "stance_no_motor"
                % Calculate the response
                [T1, Y1] = ode45(@stance_no_motor_dynamics, time, init, no_motor_options);
                init = [Y1(end, 1), Y1(end, 2)];
                T = [T; T1(1:end-1) + last_end_time];
                last_end_time = max(T1) + last_end_time;

                state = "stance_motor";
                Kinematics.Y_Position = [Kinematics.Y_Position; Y1(1:end-1, 1)];
                clear T1 Y1

            case "stance_motor"
                % Calculate the response
                [T1, Y1] = ode45(@stance_motor_dynamics, time, init, motor_options);
                init = [Y1(end, 1), Y1(end, 2)];
                T = [T; T1(1:end-1) + last_end_time];
                last_end_time = max(T1) + last_end_time;

                state = "flight";
                Kinematics.Y_Position = [Kinematics.Y_Position; Y1(1:end-1, 1)];
                clear T1 Y1


        end

    end
    Kinematics.T = T;
end

%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%%%%               EVENT FUNCTIONS                     %%%%
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

% This function is used to determine when an 'event' occurs; i.e. when
% the ball hits the ground
function [position,isterminal,direction] = flight_event_func(t,y)
    ball_radius = .02;
    position = y(1) - ball_radius; % The value that we want to be zero
    isterminal = 1;  % Halt integration
    direction = -1;   % Zero approached by decreasing values
end

function [position,isterminal,direction] = stance_no_motor_event_func(t,y)
    
    position = y(2); % The value that we want to be zero
    isterminal = 1;  % Halt integration
    direction = 0;

end

function [position,isterminal,direction] = stance_motor_event_func(t,y)
    position = y(1) -.0005; % The value that we want to be zero
    isterminal = 1;  % Halt integration
    direction = 1;   % Zero approached by increasing values
end


%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%%%%            DYNAMICS FUNCTIONS                     %%%%
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

% Dynamics of the system using the force of gravity as the only force
function func = flight_dynamics(t, y)

    g = -9.81;

    y1 = y(1);
    y2 = y(2);

    y1p = y(2);
    y2p = g;

    func = [y1p; y2p];
end

% Dynamics of the system when it is acting as a mass-spring-damper
% during stance
function func = stance_no_motor_dynamics(t,y)

    g = 9.81;
    kp = 250;
    kd = 7;
    m = .5;
    spring_natural_length =.02;
    L = spring_natural_length;

    y1 = y(1);
    y2 = y(2);

    y1p = y(2);
    y2p = kp*(L - y(1))/m - kd*y1p/m -g;

    func = [y1p; y2p];
end

function func = stance_motor_dynamics(t,y)
    % unfinished

    transmission_ratio = 1;

    km = -9550/.257; % slop of the speed-torque curve

    slope = (1/transmission_ratio)^2 * km; % slop of speed-force curve

    % Force of the motor is gonna be dependent on speed, y(2)s




    g = 9.81;
    kp = 250;
    kd = 7;
    m = .5;
    spring_natural_length = .02;
    L = spring_natural_length;
    Force = 19;

    y1 = y(1);
    y2 = y(2);

    y1p = y(2);
    y2p = kp*(L - y(1))/m - kd*y1p/m -g + Force/m;

    func = [y1p; y2p];
end

