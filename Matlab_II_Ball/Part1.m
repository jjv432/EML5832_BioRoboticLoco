clc; clear; close all; format compact

figure();
hold on
for i = 1:5


end


ball

function [position,isterminal,direction] = ball_event_func(t,y)
position = y(1); % The value that we want to be zero
isterminal = 1;  % Halt integration
direction = 0;   % The zero can be approached from either direction
end

function ball

options = odeset('Events', @ball_event_func);

time = 0:.01:5;
init = [5 0];

[T1, Y1] = ode45(@dynamics, time, init, options);
figure(1)
plot(T1, Y1(:, 1));
xlabel("Time (s)")
ylabel("Ball Height (m)")
title("Ball Height vs Time")
end


function func = dynamics(t, y)

g = -9.81;

y1 = y(1);
y2 = y(2);

y1p = y(2);
y2p = g;

func = [y1p; y2p];
end
