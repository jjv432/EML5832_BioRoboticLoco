function createSurfacePlots(params, time, x0, model_boolean)
    % x-axis is the initial z value, z_d value is the y-axis, stability is the
    % vertical

    persistent max_z_d min_z z_sample_resolution z_d_sample_resolution z_d_sample_points z_sample_points stabilities plot_count min_z_d max_z

    if isempty(plot_count)
        plot_count = 1;
    end

    if isempty(z_d_sample_points)

        % min_torque = 0;
        % max_torque = 3500;
        % min_phi_val = -pi/2;
        % max_phi_val = 0;

        min_z_d = 0;
        max_z_d = 3500;
        min_z = -pi/2;
        max_z = 0;

        z_sample_resolution = 25;
        z_d_sample_resolution = .1;
        z_d_sample_points = min_z_d:z_sample_resolution:max_z_d;
        z_sample_points = min_z:z_d_sample_resolution:max_z;
        stabilities = zeros(numel(z_d_sample_points), numel(z_sample_points));
    end


    for i = 1:numel(z_d_sample_points)
        for j = 1:numel(z_sample_points)

            cur_z_d = z_d_sample_points(i);
            cur_z = z_sample_points(j);

            % params.t_hip = cur_z_d;
            % params.phi_0 = cur_z;
            x0(3) = cur_z;
            x0(4) = cur_z_d;
            stabilities(i, j) = stabilityMeasure(params,time, x0, model_boolean);

        end
    end

    figure()
    ax = gca;
    hold on

    persistent X Y
    if isempty(X)
        [X, Y] = meshgrid(z_sample_points, z_d_sample_points);
    end

    if stabilities == zeros(numel(z_d_sample_points), numel(z_sample_points))
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