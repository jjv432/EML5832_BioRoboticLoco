clc; clear; close all;

% Parameters of the system
m1 = 2;
m2 = 10;
g = 9.8;
L = 1;
k = 1;

% Define generalized coords as sym vars and funs
syms t l(t) th(t)

%% Step 1: Gen coords
q = [l; th]; % vector of gen coords
dq = diff(q, t); % gen velos
ddq = diff(q, t, t); % gen accels

%% Step 2: Calculate Energies and Lagrangian

% Create vectors pointing to mass centers
r1 = [l; 0];
% r2 = [l + L*sin(th); -L*cos(th)];

dr1 = diff(r1, t);
% dr2 = diff(r2, t);

% Kinetic energy
T1 = (1/2)*m1*dr1.'*dr1;
% T2 = (1/2)*m2*dr2.'*dr2;

% T = T1 + T2;
T = T1;

% Potential Energy
V1 = m1*g*[0, 1]*r1 + (1/2)*k*(norm(r1) - L)^2;
% V2 = m2*g*[0, 1]*r2;

V = V1;

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
syms l_ dl_ ddl_ th_ dth_ ddth_ % non-time-varying equivalents

% Need to do a, then v, then s. Otherwise, it'll mess up the derivatives
EL_LHS = subs(EL_LHS, diff(l, t, t), ddl_);
EL_LHS = subs(EL_LHS, diff(l, t), dl_);
EL_LHS = subs(EL_LHS, l, l_);

EL_LHS = subs(EL_LHS, diff(th, t, t), ddth_);
EL_LHS = subs(EL_LHS, diff(th, t), dth_);
EL_LHS = subs(EL_LHS, th, th_);

% Need to make EL_LHS non-time-varying
EL_LHS_(1, 1) = [1 0]*EL_LHS;
EL_LHS_(2, 1) = [0 1]*EL_LHS;

% Solve for accels
ddq_ = [ddl_; ddth_];
ddq_solve = solve(EL_LHS_ == [0;0], ddq_); % store accels

% ddq_solve.dds_(1)

%% Simulate our eom
% x = [s, s_d, th, th_d]'
dds_solve = ddq_solve.ddl_;
ddth_solve = ddq_solve.ddth_;

% Create a fn that returns the acceleration of s when fed a state vector
x = [l_; dl_; th_; dth_];
dds_fun = matlabFunction(dds_solve, 'Vars', {x});
ddth_fun = matlabFunction(ddth_solve, 'Vars', {x});

% odefun for ode45
odefun = @(time, state) [state(2); dds_fun(state); state(4); ddth_fun(state)]; % derivative of states 

% sim using ode45
tspan = [0 10];
x0 = [2; 0; 0; 0];
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
s_anim = interp1(t_sim, x_sim(:, 1), t_anim);
th_anim = interp1(t_sim, x_sim(:, 3), t_anim);

figure();
for iter = 1:numel(t_anim)
    cla; % clear axes
    % plot cart as red square
    plot(s_anim(iter), 0, 'rs', 'MarkerSize', 10, 'MarkerFaceColor', 'r');
    hold on
    plot([s_anim(iter), s_anim(iter) + L*sin(th_anim(iter))], [0, -L*cos(th_anim(iter))], 'k-', 'LineWidth', 2)
    axis equal
    axis([-5 5 -2 2]);
    drawnow;

end