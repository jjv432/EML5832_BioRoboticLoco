%%
%{
    J. Vranicar Bio/Robotic Locomotion
    Hw Matlab 1B
    jjv20@fsu.edu
%}

%% Part 4
clc; clear all; close all; format compact

section4

function section4
    pendulum
end

function pendulum
    
    time = 0:.01:5;
    init = [1 0];
    
    [T1, Y1] = ode45(@undamped, time, init);
    figure(1)
    plot(T1, Y1(:, 1));
    xlabel("Time (s)")
    ylabel("Pendulum Angle (rad)")
    title("Pendulum Time Response")
end

function func = undamped(t, y)

    L = 0.30;
    g = 9.81;
    
    y1 = y(1);
    y2 = y(2);
    
    y1p = y2;
    y2p = -(g/L) * sin(y1);
    
    
    func = [y1p; y2p];

end