function createSurfacePlots(params, time, x0, model_boolean)
    % x-axis is the initial z value, z_d value is the y-axis, stability is the
    % vertical

    persistent min_phi_val max_phi_val max_x_d phi_sample_count x_d_sample_count x_d_sample_points phi_sample_points stabilities plot_count min_x_d max_z

    if isempty(plot_count)
        plot_count = 1;
    end

    if isempty(x_d_sample_points)

        min_phi_val = -pi/2;
        % min_phi_val = -1;
        max_phi_val = 0;
        
        min_x_d = 7;
        max_x_d = 15;

        phi_sample_count = 15;
        x_d_sample_count = 15;

        x_d_sample_points = linspace(min_x_d, max_x_d, x_d_sample_count);
        phi_sample_points = linspace(min_phi_val, max_phi_val, phi_sample_count);

        stabilities = zeros(numel(x_d_sample_points), numel(phi_sample_points));
    end


    for i = 1:numel(x_d_sample_points)
        for j = 1:numel(phi_sample_points)

            cur_x_d = x_d_sample_points(i);
            cur_phi = phi_sample_points(j);

            params.phi_0 = cur_phi;
            x0(2) = cur_x_d;
            stabilities(i, j) = stabilityMeasure(params,time, x0, model_boolean);

        end
    end

    figure()
    ax = gca;

    persistent X Y
    if isempty(X)
        [X, Y] = meshgrid(phi_sample_points, x_d_sample_points);
    end

    if stabilities == zeros(numel(x_d_sample_points), numel(phi_sample_points))
        fprintf("Failed to produce solution at %d\n", params.phi_0)
    else
        stabilities(stabilities > 2) = NaN;
        contourf(X, Y, real(stabilities));
        xlabel("\phi_0 (rad)");
        ylabel("V_x (m/s)");
        colorbar(ax,"eastoutside");
        clim([0 2]);
        title("Stability for " + params.phi_0);
        saveas(gcf, "StabilityPlots/Stability" + num2str(plot_count) + ".png");
    end

    plot_count = plot_count + 1;
end