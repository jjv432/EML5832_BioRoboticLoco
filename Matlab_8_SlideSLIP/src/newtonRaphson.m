function fixedPoint = newtonRaphson(params, time, x0)

    %% Newton-Rhapson

    iter = 0;
    stallIterations = 0;
    tol = 1e-6;
    del = 1e-5;
    nr_max_iter = 200;

    % x0 =[params.l0; params.l_d_0; params.phi_0; params.phi_d_0]; % this was our initial

    %{
We're comparing the initial stance parameters, so the simulation needs to
return the same parameters after the next heel strike occurs. Going to
abandon the idea of comparing the flight stances for now.
    %}

    [~, ~, R] = simulateSystem(params, time, 1, x0);
    E = R - x0;
    Error = norm(E);

    states_to_vary = [3 4];

    while (Error > tol) && (stallIterations < nr_max_iter)

        % for i = 1:numel(x0)
        for i = states_to_vary
            x0(i) = x0(i) + del;
            [~, ~,R1] = simulateSystem(params, time, 1, x0);

            x0(i) = x0(i) -2*del;
            [~, ~,R2] = simulateSystem(params, time, 1, x0);

            E2 = R2 - x0;

            x0(i) = x0(i) + 2*del;
            E1 = R1 - (x0 + del);

            [~, ~, tmp] = simulateSystem(params, time, 1, x0);
            Ex0 =  tmp - x0;
            slope(:, i) = (E1 - E2) / (2 * del);
            x0(i) = x0(i) - del;
        end
        slope = slope(states_to_vary, states_to_vary);
        x1 = x0;
        x1(states_to_vary) = x0(states_to_vary) - (slope^-1) * Ex0(states_to_vary);

        [~, ~, tmp] = simulateSystem(params, time, 1, x1);
        new_error = norm(tmp - x1);
        if new_error < Error
            Error = new_error;
            stallIterations = 0;
        else
            stallIterations = stallIterations + 1;
        end
        x0 = x1;
        clear slope

        iter = iter + 1;
    end

    fprintf("Newton-Raphson solved in %d iterations\n", iter);
    fixedPoint = x1;

end