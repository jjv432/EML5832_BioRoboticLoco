clc;
clear all;
close all;
format compact;

%% Task 1

A = magic(4);

%% Task 2

disp(A(:,3))

%% Task 3

Mean = mean(A(2, :))

%% Task 4
Sigma = std(A(2,:))

%% Task 5
DiagSum = sum(diag(A))

%% Task 6
A_Squared = A^2

%% Task 7
A_Dot_Squared = A.^2

%% Task 8
B = rand(4)

%% Task 9
B = (B + B.')/2

%% Task 10
Max_Eig = max(eig(A))

%% Task 11
OneList = ones([400 1]);

%% Task 12
MultAOnes = A * ones([4 1])