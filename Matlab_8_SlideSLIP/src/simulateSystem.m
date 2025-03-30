function [Kinematics, T, nr_results] = simulateSystem(params, time, nr_bool, init)


    % Ending Conditions for ODE45
    stance_options = odeset('Events', @(t, y) stance_event_func(t,y,params));
    flight_options = odeset('Events', @(t, y) flight_event_func(t,y,params));
    slide_options = odeset('Events', @(t, y) slide_event_func(t,y,params));
    % flight_options = [flight_options ; odeset('Events', @(t,y) flight_apex_event_func(t, y, params))];

    %% ODE45 Call
    Kinematics.X = [];
    Kinematics.Z = [];
    Kinematics.Phi = [];
    Kinematics.L = [];

    T = [];
    state = 'stance';

    % init = [params.l0; params.l_d_0; params.phi_0; params.phi_d_0];


    % If you're running newton rhapson, only run each state once
    if nr_bool
        duration = 2;
    else
        duration = 20;
    end

    for i = 1:duration

        switch state

            case 'stance'

                % Run the stance simulation
                [T1, Y1, te, ye, ie] = ode45(@(T1, Y1) stance_dynamics(T1, Y1, params), time, init, stance_options);

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
                Kinematics.X = [Kinematics.X; x_vals(1:end-1) + x_offset];
                Kinematics.Z = [Kinematics.Z; z_vals(1:end-1)];
                Kinematics.Phi = [Kinematics.Phi; phi_vals(1:end-1)];
                Kinematics.L = [Kinematics.L; l_vals(1:end-1)];
                T = [T; T1(1:end-1) + t_offset, ones(numel(T1)-1, 1)];


                % if the flight condition caused the ode to stop
                if ie == 1

                    % Set 'state' and the init vector for the next state
                    init = [x_vals(end) + x_offset; x_d_vals(end); z_vals(end); z_d_vals(end)];
                    state = 'flight';

                % If the sliding condition caused the ode to stop
                elseif ie ==2

                    % Set 'state' and the init vector for the next state
                    % For sliding, need l, l_d, phi, phi_d, x, and x_d Need
                    % x because have 3rd EOM for sliding
                    init = [l_vals(end); l_d_vals(end); phi_vals(end); phi_d_vals(end); x_vals(end); x_d_vals(end)];
                    state = 'sliding';

                end

            case 'flight'

                [T1, Y1, te, ye, ie] = ode45(@(T1, Y1) flight_dynamics(T1, Y1, params), time, init, flight_options);

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

                Kinematics.X = [Kinematics.X; x_vals(1:end-1)];
                Kinematics.Z = [Kinematics.Z; z_vals(1:end-1)];
                Kinematics.Phi = [Kinematics.Phi; zeros(numel(x_vals) - 1, 1)];
                Kinematics.L = [Kinematics.L; zeros(numel(x_vals) - 1, 1)];
                T = [T; T1(1:end-1) + t_offset, zeros(numel(T1)-1, 1)];

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

            case 'sliding'

                % Run the stance simulation
                [T1, Y1] = ode45(@(T1, Y1) slide_dynamics(T1, Y1, params), time, init, slide_options);

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
                Kinematics.X = [Kinematics.X; x_vals(1:end-1) + x_offset];
                Kinematics.Z = [Kinematics.Z; z_vals(1:end-1)];
                Kinematics.Phi = [Kinematics.Phi; phi_vals(1:end-1)];
                Kinematics.L = [Kinematics.L; l_vals(1:end-1)];
                T = [T; T1(1:end-1) + t_offset, ones(numel(T1)-1, 1)];

                % Set 'state' and the init vector for the next state
                init = [x_vals(end) + x_offset; x_d_vals(end); z_vals(end); z_d_vals(end)];
                state = 'flight';

        end

    end

    if nr_bool
        nr_results = init;
    else
        nr_results = [];
    end

end