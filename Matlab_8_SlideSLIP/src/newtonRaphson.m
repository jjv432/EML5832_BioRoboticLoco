function fixedPoint = newtonRaphson(params, time, x0)

    %% Newton-Rhapson

    iter = 0;
    stallIterations = 0;
    tol = 1e-6;
    del = 1e-6;
    nr_max_iter = 300;

    % x0 =[params.l0; params.l_d_0; params.phi_0; params.phi_d_0]; % this was our initial

    %{
We're comparing the initial stance parameters, so the simulation needs to
return the same parameters after the next heel strike occurs. Going to
abandon the idea of comparing the flight stances for now.
    %}

    [~, ~, R] = simulateSystem(params, time, 1, x0);
    E = R - x0;
    Error = norm(E);

    states_to_vary = [2, 4];

    while (Error > tol) && (stallIterations < nr_max_iter)

        % for i = 1:numel(x0)
        for i = states_to_vary
            x0(i) = x0(i) + del;
            [~, ~,R1] = simulateSystem(params, time, 1, x0);

            x0(i) = x0(i) -2*del;
            [~, ~,R2] = simulateSystem(params, time, 1, x0);

            if ~isempty(R2)
                E2 = R2 - x0;
            else
                break;
            end

            x0(i) = x0(i) + 2*del;
            if ~isempty(R1)
                E1 = R1 - (x0 + del);
            else
                break;
            end

            [~, ~, tmp] = simulateSystem(params, time, 1, x0);

            if ~isempty(tmp)

                Ex0 =  tmp - x0;
            else
                break;
            end
            slope(:, i) = (E1 - E2) / (2 * del);
            x0(i) = x0(i) - del;
        end
        slope = slope(states_to_vary, states_to_vary);
        x1 = x0;
        x1(states_to_vary) = x0(states_to_vary) - (slope^-1) * Ex0(states_to_vary);

        [~, ~, tmp] = simulateSystem(params, time, 1, x1);

        if ~isempty(tmp)
            new_error = norm(tmp - x1);
        else
            break;
        end

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