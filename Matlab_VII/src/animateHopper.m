function animateHopper(Kinematics, T)

    leg_coordinates = [0 1 1 0; -.05 -.05 .05 .05];


    X = Kinematics.X;
    Z = Kinematics.Z;
    Phi = Kinematics.Phi;


    stance_bools = T(:, 2);
    stance_x_vals = X .* stance_bools;

    mean_x_vector = [];
    
    last_stance_bool = 1;
    cur_sum = 0;
    num = 0;

    for i = 1:numel(stance_x_vals)

        stance_bool = stance_x_vals(i) ~= 0;

        if stance_bool
            cur_sum = cur_sum + stance_x_vals(i);
            num = num + 1;

        elseif stance_bool == 0 && last_stance_bool == 1
            cur_mean = cur_sum/num;
            mean_x_vector = [mean_x_vector; cur_mean];
            cur_mean = 0;
            cur_sum = 0;
            num = 0;
        end

        last_stance_bool = stance_bool;

    end


    figure();
    grid on
    hold on
    yline(0, 'k--');
    plot(0, 0, 'x', 'LineWidth', 3);
    stance_iter = 0;
    transition = 1;
    persistent stance_x_start;
    for i = 2:length(X)

        if T(i, 2)
            if transition == 1
                stance_iter = stance_iter + 1;
            end

            h1 = plot(X(i), Z(i), 'rx', 'LineWidth',5);
            theta = pi/2 - Phi(i);
            rot_matrix = [cos(theta), -sin(theta); sin(theta), cos(theta)];
            coords = rot_matrix * leg_coordinates;
            x_coords= coords(1, :);
            y_coords = coords(2, :);

            if isempty(stance_x_start)
                stance_x_start = mean_x_vector(stance_iter);
            end

            h2 = fill(x_coords + stance_x_start, y_coords, 'g');

            transition = 0;
        else
            transition = 1;
            stance_x_start = [];
            h1 = plot(X(i), Z(i), 'kx', 'LineWidth',5);
            h2 = [];
        end


        axis([-1 9 -1 9])
        axis equal
        xlabel("x-position, m")
        ylabel("z-position, m")
        legend("Ground","Origin", "Center of Mass")
        title("SLIP-Model Position")
        pause(T(i) - T(i-1));

        delete(h1)
        if ~isempty(h2)
            delete(h2);
        end
    end


end