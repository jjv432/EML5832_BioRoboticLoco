clc; clear; close all; format compact


%% 1a: Newton-Rhapson Method (const force)
% Error = 100;
tol = .01;
del = .001;
x0 = [.2];
E = simulation_const_force(x0, 0) - x0;
Error = norm(E);

while Error > tol

    for i = 1:numel(x0)
        R1 = simulation_const_force(x0 + del, 0);
        R2 = simulation_const_force(x0 - del, 0);
        E1 = R1 - (x0 + del);
        E2 = R2 - (x0 - del);
        Ex0 = simulation_const_force(x0, 0) - x0;
        slope = -(E1 - E2) / (2 * del); % will only work if this is negative...
    end

    x1 = x0 - ((slope^-1) * Ex0);

    new_error = norm(simulation_const_force(x1, 0) - x1);
    if new_error < Error
        Error = new_error;
    end
    x0 = x1;
end

x1

% Running it numerous times
simulation_const_force(x1, 1);

%% 1b: Stability of the Fixed Point (const force)

x0 = x1;

for i = 1:numel(x0)
    R1 = simulation_const_force(x0 + del, 0);
    R2 = simulation_const_force(x0 - del, 0);
    slope = (R1 - R2)/(2*del);
end

MaxEig = max(eig(slope));

%% 1c: Stability of actuation Points

% Const Force (reusing values)
disp("Constant force values");
disp(MaxEig);


%% Motor Model
clearvars;
tol = .01;
del = .001;
x0 = [1];
E = simulation_motor_actuation(x0, 0) - x0;
Error = norm(E);

while Error > tol

    for i = 1:numel(x0)
        R1 = simulation_motor_actuation(x0 + del, 0);
        R2 = simulation_motor_actuation(x0 - del, 0);
        E1 = R1 - (x0 + del);
        E2 = R2 - (x0 - del);
        Ex0 = simulation_motor_actuation(x0, 0) - x0;
        slope = -(E1 - E2) / (2 * del); % will only work if this is negative...
    end

    x1 = x0 - ((slope^-1) * Ex0);

    new_error = norm(simulation_motor_actuation(x1, 0) - x1);
    if new_error < Error
        Error = new_error;
    end
    x0 = x1;
end

% This is being returned as a negative number...
x1

% Running it numerous times
simulation_motor_actuation(x1, 1);

% Stability of the Fixed Point

x0 = x1;

for i = 1:numel(x0)
    R1 = simulation_motor_actuation(x0 + del, 0);
    R2 = simulation_motor_actuation(x0 - del, 0);
    slope = (R1 - R2)/(2*del);
end

MaxEig = max(eig(slope));

disp("Motor Model");
disp(MaxEig);

%% Variable Spring
clearvars;
tol = .001;
del = .001;
x0 = [1];
E = simulation_variable_spring(x0, 0) - x0;
Error = norm(E);

while Error > tol

    for i = 1:numel(x0)
        R1 = simulation_variable_spring(x0 + del, 0);
        R2 = simulation_variable_spring(x0 - del, 0);
        E1 = R1 - (x0 + del);
        E2 = R2 - (x0 - del);
        Ex0 = simulation_variable_spring(x0, 0) - x0;
        slope = -(E1 - E2) / (2 * del); % will only work if this is negative...
    end

    x1 = x0 - ((slope^-1) * Ex0);

    new_error = norm(simulation_variable_spring(x1, 0) - x1);
    if new_error < Error
        Error = new_error;
    end
    x0 = x1;
end

x1

% Running it numerous times
simulation_variable_spring(x1, 1);

% Stability of the Fixed Point

x0 = x1;

for i = 1:numel(x0)
    R1 = simulation_variable_spring(x0 + del, 0);
    R2 = simulation_variable_spring(x0 - del, 0);
    slope = (R1 - R2)/(2*del);
end

MaxEig = max(eig(slope));
axis auto
disp("Variable Spring");
disp(MaxEig);