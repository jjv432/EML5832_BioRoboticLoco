function maxEig = stabilityMeasure(params,time, fixedPoint, model_boolean)

    %% Stability
    slope = [];

    del = 1e-5;

    x0 = fixedPoint;

    for i = 1:numel(x0)
        x0(i) = x0(i) + del;
        [~, ~,R1] = simulateSystem(params, time, 1, x0, model_boolean);

        x0(i) = x0(i) - 2*del;
        [~, ~,R2] = simulateSystem(params, time, 1, x0, model_boolean);

        if (numel(R1) ~= 4 || numel(R2) ~=4 || numel(R1) ~= numel(R2))
            slope = [];
            break;
        end
        slope(:, i) = (R1 - R2)/(2*del);
        x0(i) = x0(i) + del;
    end

    if ~isempty(slope)
        maxEig = max(eig(slope));
    else
        maxEig = NaN;
    end

end