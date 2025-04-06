clc; clear; close all;

% Parameters of the system
m1 = 2;
m2 = 10;
g = 9.8;
L = 1;
l0 = 1;
k = 50;

% Define generalized coords as sym vars and funs
syms t x(t) l(t) th(t)

%% Step 1: Gen coords
q = [l; th; x]; % vector of gen coords
dq = diff(q, t); % gen velos
ddq = diff(q, t, t); % gen accels

%% Step 2: Calculate Energies and Lagrangian

% Create vectors pointing to mass centers
r = [x + l*cos(th); l*sin(th)];

dr = diff(r, t);

% Kinetic energy
T = (1/2)*m1*dr.'*dr;

% Potential Energy
V = m1*g*[0, 1]*r + (1/2)*k*(l-l0)^2;

% Lagrange
Lagr = T - V;

%% Step 3. Sub Lagrangian into the E-L eqn
dL_dq = gradient(Lagr, q);
dL_ddq = gradient(Lagr, dq);
dL_ddq_dt = diff(dL_ddq, t);

% LHS of EL eqn
EL_LHS = dL_ddq_dt - dL_dq;

% Solve for ddq (accelerations)
% Substitute sym vars in place of sym funs
% subs(EL_LHS, diff(s,t), ds);
syms x_ dx_ ddx_ l_ dl_ ddl_ th_ dth_ ddth_ % non-time-varying equivalents

% Need to do a, then v, then s. Otherwise, it'll mess up the derivatives
EL_LHS = subs(EL_LHS, diff(x, t, t), ddx_);
EL_LHS = subs(EL_LHS, diff(x, t), dx_);
EL_LHS = subs(EL_LHS, x, x_);

EL_LHS = subs(EL_LHS, diff(th, t, t), ddth_);
EL_LHS = subs(EL_LHS, diff(th, t), dth_);
EL_LHS = subs(EL_LHS, th, th_);

EL_LHS = subs(EL_LHS, diff(l, t, t), ddl_);
EL_LHS = subs(EL_LHS, diff(l, t), dl_);
EL_LHS = subs(EL_LHS, l, l_);

% Need to make EL_LHS non-time-varying
% EL_LHS_(1, 1) = [1 0 0]*EL_LHS;
% EL_LHS_(2, 1) = [0 1 0]*EL_LHS;
% EL_LHS_(3, 1) = [0 0 1]*EL_LHS;

% EL_LHS_(1, 1) = [1 0 0]*EL_LHS;
EL_LHS_(1, 1) = [1 0 0]*EL_LHS;
EL_LHS_(2, 1) = [0 1 0]*EL_LHS;
% EL_LHS_(3, 1) = [0 0 1]*EL_LHS;


% Solve for accels
% ddq_ = [ddx_; ddl_; ddth_];
ddq_ = [ddl_; ddth_; ddx_];
ddq_solve = solve(EL_LHS_ == [0;0], ddq_); % store accels
% ddq_solve = solve(EL_LHS_ == [0;0;0], ddq_); % store accels

% ddq_solve.dds_(1)

%% Simulate our eom
% x = [s, s_d, th, th_d]'
ddl_solve = ddq_solve.ddl_;
ddth_solve = ddq_solve.ddth_;
ddx_solve = ddq_solve.ddx_;

% Create a fn that returns the acceleration of s when fed a state vector
states = [l_; dl_; th_; dth_; x_; dx_];
ddx_fun = matlabFunction(ddx_solve, 'Vars', {states});
ddth_fun = matlabFunction(ddth_solve, 'Vars', {states});
ddl_fun = matlabFunction(ddl_solve, 'Vars', {states});

% odefun for ode45
odefun = @(time, state) [state(2); ddl_fun(state); state(4); ddth_fun(state); state(6); ddx_fun(state)]; % derivative of states 

% sim using ode45
tspan = [0 10];
x0 = [1; 0; pi/2; 0; 0; 0];
[t_sim, x_sim] = ode45(odefun, tspan, x0);

% Plot of position and angle
figure();
subplot(2, 1, 1)
plot(t_sim, x_sim(:,1));
xlabel("t (s)");
ylabel("y (m)");
subplot(2, 1, 2)
plot(t_sim, x_sim(:,3));
xlabel("t (s)");
ylabel("angle (rad)");

% Plot animation
FPS = 20;
t_anim = tspan(1):1/FPS:tspan(2);
l_anim = interp1(t_sim, x_sim(:, 1), t_anim);
th_anim = interp1(t_sim, x_sim(:, 3), t_anim);

figure();
for iter = 1:numel(t_anim)
    cla; % clear axes
    % plot cart as red square
    hold on
    plot(l_anim(iter)*cos(th_anim(iter)), l_anim(iter)*sin(th_anim(iter)), 'ro', 'MarkerSize', 10, 'MarkerFaceColor', 'r')
    yline(0, 'k');
    axis equal
    axis auto
    axis([-5 5 -2 2]);
    drawnow;

end
