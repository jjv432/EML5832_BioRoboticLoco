clc; clear; close all; format compact

%{

%}

% Human Data
scaling_factor = 0.30;

HumanData.length = 1; %m
HumanData.mass = 80; %kg
HumanData.stiffness = 20000; %N/m
HumanData.touchdown_angle = 1.2; %rad
HumanData.stride_freq = 5; %hz
HumanData.horiz_velo = 3; %m/s

ScaledHuman = dynamic_scaler(HumanData, scaling_factor);
ScaledHuman.Dampening = 3.8;

drop_height = .10;
Kinematics = ball_response(drop_height, ScaledHuman);
Ball = create_ball(.02);
draw_ball(Kinematics, ScaledHuman);

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

function draw_ball(Kinematics, ScaledHuman)

    figure(); 
    plot(Kinematics.T, Kinematics.Y_Position)
    title("Time Response of the Constant-Force Model Ball");
    xlabel("Time (s)");
    ylabel("Vertical Position (m)");
    grid on
    ylim([-.2 .2]);

    fprintf("The force value required for hopping is %.1f N\n", 5.5);

end


%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%%%%              STATE-BASED FUNCTION                 %%%%
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

function Kinematics = ball_response(initial_height, Scaled)

    Kinematics.Y_Position = [];

    state = "flight";

    time = [0, 100];
    init = [initial_height 0];
    last_end_time = 0;

    % This stops the solving when the ball is on the ground
    flight_options = odeset('Events', @flight_event_func);
    light_stance_options = odeset('Events', @stance_no_force_event_func);
    heavy_stance_options = odeset('Events', @stance_force_event_func);

    T = [];

    % Calculate the response for multiple bounces
    for i = 1:30

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

                state = "stance_no_force";
                Kinematics.Y_Position = [Kinematics.Y_Position; Y1(1:end-1, 1)]; % End of this one is beginning of the next one, values were duplicated before
                clear T1 Y1

            case "stance_no_force"
                % Calculate the response
                [T1, Y1] = ode45(@(T1, Y1) stance_no_force_dynamics(T1, Y1, Scaled), time, init, light_stance_options);
                init = [Y1(end, 1), Y1(end, 2)];
                T = [T; T1(1:end-1) + last_end_time];
                last_end_time = max(T1) + last_end_time;

                state = "stance_force";
                Kinematics.Y_Position = [Kinematics.Y_Position; Y1(1:end-1, 1)];
                clear T1 Y1

            case "stance_force"
                % Calculate the response
                [T1, Y1] = ode45(@(T1, Y1) stance_force_dynamics(T1, Y1, Scaled), time, init, heavy_stance_options);
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

function [position,isterminal,direction] = stance_no_force_event_func(t,y)
    
    position = y(2); % The value that we want to be zero
    isterminal = 1;  % Halt integration
    direction = 0;

end

function [position,isterminal,direction] = stance_force_event_func(t,y)
    position = y(1) -.02; % The value that we want to be zero
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
function func = stance_no_force_dynamics(t,y, ScaledHuman)

    g = 9.81;
    m = ScaledHuman.mass;
    kp = ScaledHuman.stiffness;
    kd = ScaledHuman.Dampening;

    spring_natural_length =.02;
    L = spring_natural_length;

    y1 = y(1);
    y2 = y(2);

    y1p = y(2);
    y2p = kp*(L - y(1))/m - kd*y1p/m -g;

    func = [y1p; y2p];
end

function func = stance_force_dynamics(t,y, ScaledHuman)

    g = 9.81;
    m = ScaledHuman.mass;
    kp = ScaledHuman.stiffness;
    kd = ScaledHuman.Dampening;

    spring_natural_length = .02;
    L = spring_natural_length;
    Force = 5.5;

    y1 = y(1);
    y2 = y(2);

    y1p = y(2);
    y2p = kp*(L - y(1))/m - kd*y1p/m -g + Force/m;

    func = [y1p; y2p];
end

%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%%%%           DYNAMIC SCALING FUNCTION                %%%%
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

function scaled_struct = dynamic_scaler(input_struct, alpha_l)

    scaled_struct.length = input_struct.length * alpha_l;
    scaled_struct.mass = input_struct.mass * (alpha_l)^3;
    scaled_struct.stiffness = input_struct.stiffness * (alpha_l)^2;
    scaled_struct.touchdown_angle = input_struct.touchdown_angle;
    scaled_struct.stride_freq = input_struct.stride_freq * (alpha_l)^(-1/2);
    scaled_struct.horiz_velo = input_struct.horiz_velo * (alpha_l)^(1/2);

end