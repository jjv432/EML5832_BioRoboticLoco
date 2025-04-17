function createSurfacePlots(params, time, x0)
    % x-axis is the initial z value, z_d value is the y-axis, stability is the
    % vertical
    persistent plotCount
    if isempty(plotCount)
        plotCount = 1;
    end
    % fixedPoint = newtonRaphson(params, time, x0);
    % fixedPointStability = stabilityMeasure(params,time, fixedPoint);
    % fixedPointZ = fixedPoint(3);
    % fixedPointZ_D = fixedPoint(4);

    % % setting up plot parameters
    % max_z_val = ceil(2*fixedPointZ);
    % max_z_d_val = ceil(2*fixedPointZ_D);

    persistent max_z_val max_z_d_val sampleResolution z_sample_points z_d_sample_points

    if isempty(z_sample_points)

        max_z_val = 2;
        max_z_d_val = 5;

        sampleResolution = .1;
        z_sample_points = 0:sampleResolution:max_z_val;
        z_d_sample_points = 0:sampleResolution:max_z_d_val;        
    end

    stabilities = zeros(numel(z_sample_points), numel(z_d_sample_points));

    for i = 1:numel(z_sample_points)
        for j = 1:numel(z_d_sample_points)

            curZ = z_sample_points(i);
            curZ_D = z_d_sample_points(j);
            samplePoint = [0; 1; curZ; curZ_D];
            stabilities(i, j) = stabilityMeasure(params,time, samplePoint);

        end
    end

    figure()
    ax = gca;
    hold on

    persistent X Y
    if isempty(X)
        [X, Y] = meshgrid(z_d_sample_points, z_sample_points);
    end

    if numel(stabilities)<4
        fprintf("Failed to produce solution at %d\n", params.phi_0)
    else

        contourf(X, Y, real(stabilities));
        ylabel("Z Vals");
        xlabel("Z_D Vals");
        colorbar(ax,"eastoutside");
        title("Stability for " + params.phi_0);
        saveas(gcf, "Stability" + plotCount + ".png");

    end

    plotCount = plotCount + 1;

end