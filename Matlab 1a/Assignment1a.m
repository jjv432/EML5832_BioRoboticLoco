%% Part 1

%% Task 1

my_age = 22;

C = magic(4);
magic_sum = sum(C);
magic_sum = magic_sum(1);

C = C./magic_sum;

C = C.*my_age;
sum(C)

%% Task 2

Age_Vector = [0:my_age:9*my_age]';

%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

%% Part 2

x = linspace(0, 5, 10000);

y = x.^3 - 3.*x - 2;

figure()
plot(x, y)
xlim([0 5])
hold on
x = round(x, 1);
scatter(x(mod(x, .5) == 0), y(mod(x, .5) == 0))