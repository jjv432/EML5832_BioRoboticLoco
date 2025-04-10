function maxEig = stabilityMeasure(params,time, fixedPoint)

    %% Stability

    del = 1e-5;
    
    x0 = fixedPoint;


    for i = 1:numel(x0)
        x0(i) = x0(i) + del;
        [~, ~,R1] = simulateSystem(params, time, 1, x0);
        x0(i) = x0(i) - 2*del;
        [~, ~,R2] = simulateSystem(params, time, 1, x0);
        slope(:, i) = (R1 - R2)/(2*del);
        x0(i) = x0(i) + del;
    end

    maxEig = max(eig(slope));

end