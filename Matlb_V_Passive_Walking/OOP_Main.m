clc; clear; close all

walker_options = odeset('Events', @walker_event);

init = [0.1995; -.199; .3991; -.0156];
time = [0 4];
gamma = 0.009;

pw = passive_walker(init);



for i = 1:3
    pw.GenerateDynamics(time, gamma, walker_options)
    pw.AnimateWalker
end



figure()
hold on
plot(pw.T, pw.Y(:, 1))
plot(pw.T, pw.Y(:, 3))


