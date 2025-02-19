clc; clear; close all

init = [0.1995; -.199; .3991; -.0156];
time = [0 4];
gamma = 0.009;

pw = passive_walker(init);

pw.GenerateDynamics(time, gamma)
pw.AnimateWalker



figure()
hold on
plot(pw.T, pw.Y(:, 1))
plot(pw.T, pw.Y(:, 3))