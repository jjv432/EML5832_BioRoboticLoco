function animateHopper(Kinematics, T)

    X = Kinematics.X;
    Z = Kinematics.Z;
    figure();
    grid on
    hold on
    yline(0, 'k--');
    plot(0, 0, 'x', 'LineWidth', 3);
    for i = 2:length(X)
        h1 = plot(X(i), Z(i), 'rx', 'LineWidth',5);
        axis([-4 9 -4 9])
        axis equal
        xlabel("x-position, m")
        ylabel("z-position, m")
        legend("Ground","Origin", "Center of Mass")
        title("SLIP-Model Position")
        pause(T(i) - T(i-1));

        delete(h1)
    end


end