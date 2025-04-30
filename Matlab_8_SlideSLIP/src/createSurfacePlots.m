function createSurfacePlots(params, time, x0)
    % x-axis is the initial z value, z_d value is the y-axis, stability is the
    % vertical

    persistent max_torque min_phi_val torque_sample_resolution phi_sample_resolution torque_sample_points phi_sample_points stabilities plot_count min_torque max_phi_val

    if isempty(plot_count)
        plot_count = 1;
    end

    if isempty(torque_sample_points)

        min_torque = 0;
        max_torque = 3500;
        min_phi_val = -pi/2;
        max_phi_val = 0;

        torque_sample_resolution = 50;
        phi_sample_resolution = .1;
        torque_sample_points = min_torque:torque_sample_resolution:max_torque;
        phi_sample_points = min_phi_val:phi_sample_resolution:max_phi_val;
        stabilities = zeros(numel(torque_sample_points), numel(phi_sample_points));
    end


    for i = 1:numel(torque_sample_points)
        for j = 1:numel(phi_sample_points)

            cur_torque = torque_sample_points(i);
            cur_phi = phi_sample_points(j);

            params.t_hip = cur_torque;
            params.phi_0 = cur_phi;
            stabilities(i, j) = stabilityMeasure(params,time, x0);

        end
    end

    figure()
    ax = gca;
    hold on

    persistent X Y
    if isempty(X)
        [X, Y] = meshgrid(phi_sample_points, torque_sample_points);
    end

    if stabilities == zeros(numel(torque_sample_points), numel(phi_sample_points))
        fprintf("Failed to produce solution at %d\n", params.phi_0)
    else

        % curPhiDenom = (1/params.phi_0) * pi;
        % curPhiDenom = ceil(curPhiDenom * 100);
        contourf(X, Y, real(stabilities));
        ylabel("Z Vals");
        xlabel("Z_D Vals");
        colorbar(ax,"eastoutside");
        clim([0 2]);
        title("Stability for " + params.phi_0);
        saveas(gcf, "StabilityPlots/Stability" + num2str(plot_count) + ".png");
    end

    plot_count = plot_count + 1;
end