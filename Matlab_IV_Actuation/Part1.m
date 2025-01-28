clc; clear all; close all; format compact

%{
This script contains a function which allows for dynamic scaling by some
known factor, and demonstrates the function being used

jjv20@fsu.edu
%}

scaling_factor = 0.20;

HumanData.length = 1; %m
HumanData.mass = 80; %kg
HumanData.stiffness = 20000; %N/m
HumanData.touchdown_angle = 1.2; %rad
HumanData.stride_freq = 5; %hz
HumanData.horiz_velo = 3; %m/s

ScaledHuman = dynamic_scaler(HumanData, scaling_factor)


function scaled_struct = dynamic_scaler(input_struct, alpha_l)

scaled_struct.length = input_struct.length * alpha_l;
scaled_struct.mass = input_struct.mass * (alpha_l)^3;
scaled_struct.stiffness = input_struct.stiffness * (alpha_l)^2;
scaled_struct.touchdown_angle = input_struct.touchdown_angle;
scaled_struct.stride_freq = input_struct.stride_freq * (alpha_l)^(-1/2);
scaled_struct.horiz_velo = input_struct.horiz_velo * (alpha_l)^(1/2);

end