clc; clear; close all

walker_options = odeset('Events', @walker_event);

init = [0.1995; -.199; .3991; -.0156];
time = [0 5];
gamma = 0.009;

pw = passive_walker(init);

%% Just Making the First Plot
pw.GenerateDynamics(time, gamma, [])
figure()
hold on
grid on
plot(pw.T, pw.Y(:, 1))
plot(pw.T, pw.Y(:, 3))
xlabel("Time (s)")
ylabel("Angle (rad)");
legend("\theta", "\phi");
title("\gamma 0.009 rad");