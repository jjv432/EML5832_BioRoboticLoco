function final_height = simulation_motor_actuation(x0, plotting)
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

    drop_height = x0;
    Kinematics = ball_response(drop_height, ScaledHuman, plotting);
    Ball = create_ball(.02);
    final_height = draw_ball(Kinematics, plotting);


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

    function final_height = draw_ball(Kinematics, plotting)

        if plotting
            figure();
            hold on
            plot(Kinematics.T, Kinematics.Y_Position)
            title("Time Response of the Motor Model Ball");
            xlabel("Time (s)");
            ylabel("Vertical Position (m)");
            grid on
            ylim([-.2 .2]);
        end

        final_height = Kinematics.Y_Position(islocalmax(Kinematics.Y_Position));


        % fprintf("The transmission ratio selected was %d, and the maximum hopping height was %.2f m\n", 550, 5.98);
        % fprintf("This is not realistic because it requires the motor to be continuously operating outside of its continuous operating range and assumes no mechanical or electrical losses\n");

    end


    %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
    %%%%              STATE-BASED FUNCTION                 %%%%
    %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

    function Kinematics = ball_response(initial_height, Scaled, plotting)

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

        if plotting
            plotting_duration = 100;
        else
            plotting_duration = 4;
        end
        % Calculate the response for multiple bounces
        for i = 1:plotting_duration

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
                    [T1, Y1] = ode45(@(T1, Y1) stance_no_motor_dynamics(T1, Y1, Scaled), time, init, no_motor_options);
                    init = [Y1(end, 1), Y1(end, 2)];
                    T = [T; T1(1:end-1) + last_end_time];
                    last_end_time = max(T1) + last_end_time;

                    state = "stance_motor";
                    Kinematics.Y_Position = [Kinematics.Y_Position; Y1(1:end-1, 1)];
                    clear T1 Y1

                case "stance_motor"
                    % Calculate the response
                    [T1, Y1] = ode45(@(T1, Y1) stance_motor_dynamics(T1, Y1, Scaled), time, init, motor_options);
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
    function func = stance_no_motor_dynamics(t,y, ScaledHuman)

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

    function func = stance_motor_dynamics(t,y, ScaledHuman)

        transmission_ratio = 550;
        Z = 1/transmission_ratio;

        km = -9550/.257; % slop of the speed-torque curve
        t_stall = 0.257;

        % By algebra based on the motor chart...
        v = y(2);
        Force = (v + Z*km*t_stall) / (Z^2 * km);

        g = 9.81;
        m = ScaledHuman.mass;
        kp = ScaledHuman.stiffness;
        kd = ScaledHuman.Dampening;

        spring_natural_length = .02;
        L = spring_natural_length;

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
end