%%
%{
    J. Vranicar Bio/Robotic Locomotion
    Hw Matlab 1A
    jjv20@fsu.edu
%}

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

x = linspace(0, 5, 500);

y = x.^3 - 3.*x - 2;

search_xs = 0:.5:5;

y_interp = interp1(x, y, search_xs);

[~, min_index] = min(y);
[~, max_index] = max(y);

figure()
    grid on
    hold on
    plot(x, y)
    scatter(search_xs, y_interp, 'g*')
    scatter(x(min_index), y(min_index));
    scatter(x(max_index), y(max_index));
    xlim([-2 7])



%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

%% Part 3

tempers = ['1', '0', '2', '4'];
pow_count = 0;
total = 0;

for i = numel(tempers):-1:1
    cur_char = tempers(i);
    switch(cur_char)
        case '0'
            number = 0;
        case '1'
            number = 1;
        case '2'
            number = 2;
        case '3'
            number = 3;
        case '4'
            number = 4;
        case '5'
            number = 5;
        case '6'
            number = 6;
        case '7'
            number = 7;
        case '8'
            number = 8;
        case '9'
            number = 9;
    end

    addition = number * 10^pow_count;

    total = total + addition;
    pow_count = pow_count + 1;

end
total
total = total/4

%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

