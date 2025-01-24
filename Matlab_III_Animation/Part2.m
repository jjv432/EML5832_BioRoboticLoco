clc; clear; close all; format compact

%{
    This is a simple animation of the ball bouncing using the spring-mass
    -damper model of ground contact
%}

plotBool = 0;
animationBool = 1;
drop_height = 15;

Kinematics = ball_response(drop_height, plotBool);

Ball = create_ball(.2);
draw_ball(Ball, Kinematics, drop_height, animationBool);

%% Functions

function Ball = create_ball(radius)

    thetas = linspace(0, 2*pi, 100);
    x_points = radius * cos(thetas);
    y_points = radius * sin(thetas);
    Ball.X_Points = x_points;
    Ball.Y_Points = y_points;

end

function draw_ball(Ball, Kinematics, drop_height, animationBool)
    figure();
    axis equal

    hold on
    grid on


    title("Spring Mass Damper Response of a Ball");

    fps = 60;
    if animationBool
        vid = VideoWriter("MatlabIII_Pt2.avi");
        vid.FrameRate = fps;
        vid.Quality = 85;
        open(vid);
    end

    h1 = [];

    Energies = energy_calc(Kinematics);

    for i = 1:fps*10 %Ensures video is 10 seconds long
        cla(h1);

        h1 = subplot(1, 3, 1);
        fill(Ball.X_Points, Ball.Y_Points + Kinematics.Y_Position(i), 'r');
        axis([-2 2 -1 drop_height + 1])
        axis equal
        yline(0, 'k');
        xlabel("Ball Horizontal Position (m)")
        ylabel("Ball Vertical Position (m)")
        grid on

        subplot(1, 3, 2);
        hold on
        grid on
        plot(linspace(0, i/fps, i), Kinematics.Y_Position(1:i), 'k');
        h2 = scatter(i/fps, Kinematics.Y_Position(i), 'r*');
        title("Ball Trajectory vs Time");
        xlabel("Time (s)");
        ylabel("Ball Vertical Position (m)")
        axis([0 i/fps -1 drop_height + 1])

        subplot(1, 3, 3)
        hold on
        grid on
        plot(linspace(0, i/fps, i), Energies.Potential_Y(1:i), 'b')
        plot(linspace(0, i/fps, i), Energies.Kinetic_Y(1:i), 'r')
        plot(linspace(0, i/fps, i), Energies.Total_Energy(1:i), '--k')
        legend("Potential Energy", "Kinetic Energy", "Total Energy", 'location', 'NorthOutside')
        xlabel("Time(s)")
        ylabel("Energy (J)");

        drawnow;
        if animationBool
            writeVideo(vid, getframe(gcf));
        end
        delete(h2);

    end
    if animationBool
        close(vid);
    end
end

function Energies = energy_calc(Kinematics)
    m = .5;
    kp = 750;
    r = .2;

    for k = 1:numel(Kinematics.Y_Position)


        Grav_Potential_Y(k) = Kinematics.Y_Position(k) * 9.81 * m; % .5 is mass

        if Kinematics.Y_Position(k) <= r
            Spring_Potential_Y(k) = (1/2) * kp * (r - Kinematics.Y_Position(k))^2;
        else
            Spring_Potential_Y(k) = 0;
        end

        Kinetic_Y(k) = (1/2) * m * Kinematics.Y_Velocity(k)^2;


    end
    Energies.Kinetic_Y = Kinetic_Y;
    Energies.Potential_Y = Grav_Potential_Y + Spring_Potential_Y;
    Energies.Total_Energy = Kinetic_Y + Grav_Potential_Y + Spring_Potential_Y;

end

function Kinematics = ball_response(initial_height, plotBool)

    Kinematics.Stance.Y_Position = [];
    Kinematics.Stance.Y_Velocity = [];
    Kinematics.Flight.Y_Position = [];
    Kinematics.Flight.Y_Velocity = [];
    Kinematics.Y_Position = [];
    Kinematics.Y_Velocity = [];

    state = "flight";

    ball_radius = .2;
    spring_natrual_length = ball_radius;

    if plotBool
        figure();
        hold on
        grid on
    end

    time = 0:.01:5;
    init = [initial_height 0];
    last_end_time = 0;

    % This stops the solving when the ball is on the ground
    flight_options = odeset('Events', @flight_event_func);
    stance_options = odeset('Events', @stance_event_func);

    % Calculate the response for multiple bounces
    for i = 1:9

        switch(state)

            case "flight"
                % Calculate the response
                [T1, Y1] = ode45(@flight_dynamics, time, init, flight_options);

                % Plot the response
                if plotBool
                    h1 = plot(T1 + last_end_time, Y1(:, 1), 'b--', 'DisplayName', 'Flight');
                    xlabel("Time (s)")
                    ylabel("Ball Height at Center (m)")

                    title("Ball Height vs Time")
                end

                % Set the new initial value for the next solving.  Algebraically, the
                % velocity is determined.  Last_end_time is used for plotting the
                % bounces sequentially
                init = [Y1(end, 1), Y1(end, 2)];
                last_end_time = max(T1) + last_end_time;
                state = "stance";
                Kinematics.Y_Position = [Kinematics.Y_Position; Y1(:, 1)];
                Kinematics.Y_Velocity = [Kinematics.Y_Velocity; Y1(:, 2)];
                clear T1 Y1

            case "stance"
                % Calculate the response
                [T1, Y1] = ode45(@stance_dynamics, time, init, stance_options);

                % Plot the response
                if plotBool
                    h2 = plot(T1 + last_end_time, Y1(:, 1), 'r--', 'DisplayName', 'Stance');
                    xlabel("Time (s)")
                    ylabel("Ball Height at Center (m)")

                    title("Ball Height vs Time")
                end
                % Set the new initial value for the next solving.  Algebraically, the
                % velocity is determined.  Last_end_time is used for plotting the
                % bounces sequentially
                init = [Y1(end, 1), Y1(end, 2)];
                last_end_time = max(T1) + last_end_time;
                state = "flight";
                Kinematics.Y_Position = [Kinematics.Y_Position; Y1(:, 1)];
                Kinematics.Y_Velocity = [Kinematics.Y_Velocity; Y1(:, 2)];
                clear T1 Y1


        end

    end
    if plotBool
        legend([h2 h1], "Stance", "Flight");
    end

    % This function is used to determine when an 'event' occurs; i.e. when
    % the ball hits the ground
    function [position,isterminal,direction] = flight_event_func(t,y)
        position = y(1) - ball_radius; % The value that we want to be zero
        isterminal = 1;  % Halt integration
        direction = -1;   % Zero approached by decreasing values
    end

    function [position,isterminal,direction] = stance_event_func(t,y)
        position = y(1) - ball_radius; % The value that we want to be zero
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

    % Dynamics of the system when it is acting as a mass-spring-damper
    % during stance
    function func = stance_dynamics(t,y, K)
        % global K userInputKBool

        g = 9.81;
        % if userInputKBool
        %     kp = K.kp;
        %     kd = K.kd;
        % else
        kp = 750;
        kd = 15;
        % end
        m = .5;
        L = spring_natrual_length;

        y1 = y(1);
        y2 = y(2);

        y1p = y(2);
        y2p = kp*(L - y(1))/m - kd*y1p/m -g;

        func = [y1p; y2p];
    end

end