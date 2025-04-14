function animateHopper(Kinematics, T)

    leg_width = .05;
    leg_coordinates = [-leg_width/2 -leg_width/2 leg_width/2 leg_width/2; 0 -1 -1 0];


    X = Kinematics.X;
    Z = Kinematics.Z;
    Phi = Kinematics.Phi;
    L = Kinematics.L;

    figure();
    grid on
    hold on
    yline(0, 'k--');
    plot(0, 0, 'x', 'LineWidth', 3);
    

    for i = 2:length(X)

        if T(i, 2) || T(i, 3) % in stance or sliding

            h1 = plot(X(i), Z(i), 'ro', 'LineWidth',5);
            theta = -Phi(i);
            rot_matrix = [cos(theta), -sin(theta); sin(theta), cos(theta)];
            coords = rot_matrix * (leg_coordinates.*[1; L(i)]);
            x_coords= coords(1, :) + X(i);
            y_coords = coords(2, :) + Z(i);

            h2 = fill(x_coords, y_coords, 'g');

        else %flight
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