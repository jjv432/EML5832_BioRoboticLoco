function [Kinematics, T, nr_results] = simulateSystem(params, time, nr_bool, init_init)


    num_flight_runs = 3;
    % Ending Conditions for ODE45
    stance_options = odeset('Events', @(t, y) stance_event_func(t,y,params));
    flight_options = odeset('Events', @(t, y) flight_event_func(t,y,params));
    slide_options = odeset('Events', @(t, y) slide_event_func(t,y,params));

    %% ODE45 Call

    % Setting empty arrays for all the data that we want to store
    Kinematics.X = [];
    Kinematics.S = [];
    Kinematics.Z = [];
    Kinematics.Phi = [];
    Kinematics.L = [];

    apex_state = [];
    flight_counter = 0;
    T = [];

    state = 'flight';
    init = init_init;

    % If you're running newton rhapson, only run each state once
    if nr_bool
        duration = 12;
    else
        duration = 20;
    end

    % Running 'duration' number of states
    for i = 1:duration

        switch state

            case 'stance'

                % Run the stance simulation
                [T1, Y1, ~, ~, ie] = ode45(@(T1, Y1) stance_dynamics(T1, Y1, params), time, init, stance_options);

                % Parse out the results
                l_vals = Y1(:, 1);
                l_d_vals = Y1(:, 2);
                phi_vals = Y1(:, 3);
                phi_d_vals = Y1(:, 4);

                % Offset values
                % Only apply the offset if there's been a state before this
                % one
                if ~isempty(Kinematics.X)
                    x_offset = Kinematics.X(end);
                    t_offset = T(end, 1);
                else
                    x_offset = 0;
                    t_offset = 0;
                end

                % Solve for x and z positions and velocities
                x_vals = l_vals .* sin(phi_vals);
                z_vals = l_vals .* cos(phi_vals);

                % Normalize the x position
                x_vals = x_vals - x_vals(1);                

                % Store the T, x, and z positions for plotting
                Kinematics.X = [Kinematics.X; x_vals + x_offset];
                Kinematics.Z = [Kinematics.Z; z_vals];
                Kinematics.Phi = [Kinematics.Phi; phi_vals];
                Kinematics.L = [Kinematics.L; l_vals];
                T = [T; T1 + t_offset, ones(numel(T1), 1), zeros(numel(T1), 1)];

                % if the flight condition caused the ode to stop
                if ie == 1

                    % Set 'state' and the init vector for the next state
                    x_d_init = l_d_vals(end)*sin(phi_vals(end)) + phi_d_vals(end)*l_vals(end)*cos(phi_vals(end)); %* last cos switched from sin
                    z_d_init = l_d_vals(end)*cos(phi_vals(end)) + phi_d_vals(end)*l_vals(end)*sin(phi_vals(end)); %*

                    init = [x_vals(end) + x_offset; x_d_init; z_vals(end); z_d_init];
                    state = 'flight';

                    % If the sliding condition caused the ode to stop
                elseif ie ==2

                    % Set 'state' and the init vector for the next state
                    % For sliding, need l, l_d, phi, phi_d, x, and x_d Need
                    % x because have 3rd EOM for sliding
                    init = [l_vals(end); l_d_vals(end); phi_vals(end); phi_d_vals(end); x_vals(end) + l_vals(end)*sin(phi_vals(end)); 0];
                    state = 'sliding';

                end

            case 'flight'

                [T1, Y1, te, ye, ie] = ode45(@(T1, Y1) flight_dynamics(T1, Y1, params), time, init, flight_options);
                
                flight_counter = flight_counter + 1;

                if flight_counter < num_flight_runs
                    apex_state = init_init + .1;
                end

                if flight_counter == num_flight_runs && ~isempty(ye)
                    apex_state = ye(1, :)';
                end
                % Parse out the results
                x_vals = Y1(:, 1);
                x_d_vals = Y1(:, 2);
                z_vals = Y1(:, 3);
                z_d_vals = Y1(:, 4);

                % apex_height = max(z_vals);
                % if flight_counter < 3
                %     heights(flight_counter) = apex_height;
                % elseif flight_counter == 3
                %     dH = heights(2) - heights(1);
                % end
                % flight_counter = flight_counter + 1;


                % Store the T, x, and z positions for plotting
                if ~isempty(Kinematics.X)
                    x_offset = Kinematics.X(end);
                    t_offset = T(end, 1);
                else
                    x_offset = 0;
                    t_offset = 0;
                end

                % normalize the x_vals
                x_vals = x_vals - x_vals(1);

                Kinematics.X = [Kinematics.X; x_vals(1:end) + x_offset];
                Kinematics.Z = [Kinematics.Z; z_vals(1:end)];
                Kinematics.Phi = [Kinematics.Phi; zeros(numel(x_vals), 1)];
                Kinematics.L = [Kinematics.L; zeros(numel(x_vals), 1)];
                T = [T; T1(1:end) + t_offset, zeros(numel(T1), 1), zeros(numel(T1), 1)]; % This is helpful to keep track of state based on each column

                l0 = params.l0;
                vx = x_d_vals(end);
                vz = z_d_vals(end);

                % Find init for the next state
                % l_d_init = vz * cos(phi_vals(end)) + vx * sin(phi_vals(end));
                l_d_init = vz * cos(params.phi_0) + vx * sin(params.phi_0);

                % phi_d_init = sqrt((vz*sin(phi_vals(end))/l0)^2 + (vx*cos(phi_vals(end))/l0)^2);
                % phi_d_init = (vz*sin(phi_vals(end)))/l0 - (vx*cos(phi_vals(end)))/l0;
                
                
                phi_d_init = l0*(vz*sin(params.phi_0) + vx*cos(params.phi_0));
                

                init = [l0; l_d_init; params.phi_0; phi_d_init];
                stance_init = init;
                state = 'stance';

            case 'sliding'

                % Run the stance simulation
                [T1, Y1, ~, ~, ie] = ode45(@(T1, Y1) slide_dynamics(T1, Y1, params), time, init, slide_options);

                % Parse out the results
                l_vals = Y1(:, 1);
                l_d_vals = Y1(:, 2);
                phi_vals = Y1(:, 3);
                phi_d_vals = Y1(:, 4);
                s_vals = Y1(:, 5);
                s_d_vals = Y1(:, 6);


                % Offset values
                if ~isempty(Kinematics.X)
                    x_offset = Kinematics.X(end);
                    t_offset = T(end, 1);
                else
                    x_offset = 0;
                    t_offset = 0;
                end

                % Solve for x and z positions and velocities
                x_vals = l_vals .* sin(phi_vals);
                z_vals = l_vals .* cos(phi_vals);

                % Normalize the x_vals
                x_vals = x_vals - x_vals(1);

                % Store the T, x, and z positions for plotting
                % Kinematics.X = [Kinematics.X; x_vals(1:end) + x_offset + s_vals(1:end)- s_vals(1)];
                Kinematics.X = [Kinematics.X; x_vals + x_offset];
                Kinematics.Z = [Kinematics.Z; z_vals(1:end)];
                Kinematics.Phi = [Kinematics.Phi; phi_vals(1:end)];
                Kinematics.L = [Kinematics.L; l_vals(1:end)];
                T = [T; T1(1:end) + t_offset, zeros(numel(T1), 1), ones(numel(T1), 1)];

                if ie == 1
                    % Set 'state' and the init vector for the next state
                    x_d_init = l_d_vals(end)*sin(phi_vals(end)) + phi_d_vals(end)*l_vals(end)*cos(phi_vals(end)) + s_d_vals(end);
                    z_d_init = l_d_vals(end)*cos(phi_vals(end)) + phi_d_vals(end)*l_vals(end)*sin(phi_vals(end));
                    x_init = l_vals(end)*sin(phi_vals(end)) + x_offset;
                    z_init = l_vals(end)*cos(phi_vals(end));
                    init = [x_init; x_d_init; z_init; z_d_init];
                    state = 'flight';

                elseif ie ==2
                    init = [l_vals(end); l_d_vals(end); phi_vals(end); phi_d_vals(end)];
                    stance_init = init;
                    state = 'stance';
                end

        end

    end

    if nr_bool
        nr_results = apex_state;
    else
        nr_results = [];
    end

end