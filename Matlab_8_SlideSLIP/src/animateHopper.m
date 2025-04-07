function animateHopper(Kinematics, T)

    %{
mean_x_vector is producing the correct values (checked against
stance_x_vals). Something about the animation
    %}

    leg_width = .05;
    leg_coordinates = [0 1 1 0; -leg_width/2 -leg_width/2 leg_width/2 leg_width/2];


    X = Kinematics.X;
    X_Slide = Kinematics.X_Slide;
    Z = Kinematics.Z;
    Phi = Kinematics.Phi;
    L = Kinematics.L;

    stance_bools = T(:, 2);
    stance_x_vals = X .* stance_bools;

    mean_x_vector = [];

    last_stance_bool = 1;
    cur_sum = 0;
    num = 0;

    for i = 1:numel(stance_x_vals)

        stance_bool = stance_bools(i);

        if stance_bool
            cur_sum = cur_sum + stance_x_vals(i);
            num = num + 1;

        elseif stance_bool == 0 && last_stance_bool == 1
            cur_mean = cur_sum/(num);
            mean_x_vector = [mean_x_vector; cur_mean];
            cur_mean = 0;
            cur_sum = 0;
            num = 0;


        end

        last_stance_bool = stance_bool;

    end


    % special case that you're only animating one stance
    if isempty(mean_x_vector)
        mean_x_vector = mean(stance_x_vals);
    end

    figure();
    grid on
    hold on
    yline(0, 'k--');
    plot(0, 0, 'x', 'LineWidth', 3);
    stance_iter = 0;
    transition = 1;


    persistent stance_x_start;
    

    for i = 2:5:length(X)

        if T(i, 2) % in stance
            if transition == 1
                stance_iter = stance_iter + 1;
            end

            h1 = plot(X(i), Z(i), 'ro', 'LineWidth',5);
            theta = pi/2 - Phi(i);
            rot_matrix = [cos(theta), -sin(theta); sin(theta), cos(theta)];
            coords = rot_matrix * (leg_coordinates.*[L(i); 1]);
            x_coords= coords(1, :);
            y_coords = coords(2, :);

            if isempty(stance_x_start)
                stance_x_start = mean_x_vector(stance_iter);
            end

            h2 = fill(x_coords + stance_x_start, y_coords, 'g');

            transition = 0;

        elseif T(i, 3) % Sliding
            if transition == 1
                stance_iter = stance_iter + 1;
            end

            h1 = plot(X(i), Z(i), 'mo', 'LineWidth',5);
            theta = pi/2 - Phi(i);
            rot_matrix = [cos(theta), -sin(theta); sin(theta), cos(theta)];
            coords = rot_matrix * (leg_coordinates.*[L(i); 1]);
            x_coords= coords(1, :);
            y_coords = coords(2, :);

            if isempty(stance_x_start)
                stance_x_start = mean_x_vector(stance_iter);
            end

            h2 = fill(x_coords + stance_x_start + X_Slide(i) + .5, y_coords, 'g'); % for some reason .5 fixes the animation- need to figure out where the actual issue is still

            transition = 0;

        else %flight
            transition = 1;
            stance_x_start = [];
            h1 = plot(X(i), Z(i), 'ko', 'LineWidth',5);
            h2 = [];
        end


        axis([X(i)-4, X(i)+4, -1, 3])
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