% Passive Walker - Shane Rober - 02/19/2025
clc; clear; close all;

%%
iter = 0;
stallIterations = 0;
tol = .00001;
del = .00001;
x0 =[0.1995; -0.1990; 0.3991; -0.0156];

E = passive_walking(x0, 0) - x0;
Error = norm(E);

while (Error > tol) && (stallIterations < 100)

    for i = 1:numel(x0)
        x0(i) = x0(i) + del;
        R1 = passive_walking(x0, 0);

        x0(i) = x0(i) -2*del;
        R2 = passive_walking(x0, 0);

        E2 = R2 - (x0);

        x0(i) = x0(i) + 2*del;
        E1 = R1 - (x0 + del);
        
        Ex0 = passive_walking(x0, 0) - x0;
        slope(:, i) = (E1 - E2) / (2 * del); % will only work if this is negative...
        x0(i) = x0(i) - del;
    end
    x1 = x0 - (slope^-1) * Ex0;

    new_error = norm(passive_walking(x1, 0) - x1);
    if new_error < Error
        Error = new_error;
        stallIterations = 0;
    else
        stallIterations = stallIterations + 1;
    end
    x0 = x1;
    clear slope

    iter = iter + 1;
end

fprintf("Newton-Raphson solved in %d iterations", iter);
% x1

% Running it numerous times
passive_walking(x1, 1);

%% 1b: Stability of the Fixed Point

x0 = x1;

for i = 1:numel(x0)
    x0(i) = x0(i) + del;
    R1 = passive_walking(x0, 0);
    x0(i) = x0(i) - 2*del;
    R2 = passive_walking(x0, 0);
    slope(:, i) = (R1 - R2)/(2*del);
    x0(i) = x0(i) + del;
end

MaxEig = max(eig(slope))


%% Passive Walker Simulation Functions
%% EOM
% Initial Conditions (angle and angular velocities)
% init = [0.1995   -0.1990    0.3991   -0.0156];
% new_state = passive_walking(init);

function state = passive_walking(x0, plotBool)
    % Constants
    g = 9.81;      % Gravity (m/s^2)
    gamma = 0.0091; % Slope angle (radians)

    % Time span for integration
    tspan = [0, 5]; % Arbitrary, will stop at heelstrike

    % Initialize state
    state = x0;
    if plotBool
        steps = 10;
    else
        steps = 2; % Number of steps to simulate
    end
    results = []; % Store trajectory
    times = [];

    % Simulate multiple steps
    for i = 1:steps
        % Solve ODE until heelstrike
        options = odeset('Events', @heelstrike_event);
        [t, y] = ode45(@(t, x) passive_walker_ode(t, x, gamma), tspan, state, options);

        % Store results
        results = [results; y];
        if i == 1
            times = [times; t];
        else
            times = [times; times(end) + t];
        end

        % Apply heelstrike transition
        state = heelstrike_transition(y(end, :));
    end

    if plotBool
        % Plot results
        figure;
        plot(times, results(:, 1), 'r', 'LineWidth', 1.5); hold on;
        plot(times, results(:, 3), 'b', 'LineWidth', 1.5);
        xlabel('Time (s)');
        ylabel('Angles (rad)');
        legend('Stance Leg (\theta)', 'Swing Leg (\phi)');
        title('Passive Walking Simulation');
        grid on;
    end
end

%  Equations of Motion
function dxdt = passive_walker_ode(~, y, gamma)
    theta = y(1);
    phi = y(3);
    dtheta = y(2);
    dphi = y(4);

    % Equations of motion
    ddtheta = sin(theta - gamma);
    ddphi = sin(theta - gamma) + dtheta^2 * sin(phi) - cos(theta - gamma)*sin(phi);

    % Output first-order system
    dxdt = [dtheta; ddtheta; dphi; ddphi];
end

% Heelstrike Event Function
function [value, isterminal, direction] = heelstrike_event(~, y)
    value = y(3) - 2 * y(1);
    isterminal = 1; % Stop integration when event occurs
    direction = 1; % Only detect when crossing from positive to negative
end


% Heelstrike Transition Function
function new_state = heelstrike_transition(state)
    theta = state(1);
    dtheta = state(2);
    dphi = state(4);

    % Full heelstrike transition matrix from the paper
    H = [ -1   0   0   0;
        0  cos(2*theta)   0   0;
        -2   0   0   0;
        0  cos(2*theta)*(1 - cos(2*theta))   0   1];

    % Swap stance and swing legs with updated velocities
    new_state = H * [state(1);state(2);state(3);state(4)];
end

