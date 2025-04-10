function params = getParams()

    % Parameters
    params.l0 = 1;  %* m
    params.b = 10;  %*
    params.k = 100; %*
    params.g = 9.81;
    params.m = 1;  %*
    params.t_hip = 2.9;  %*
    params.phi_0 = -pi/6;  %*
    params.phi_d_0 = 1.5;
    params.l_d_0 = -4;
    params.muS = .55; % Good for sliding
    params.muK = (.75)*params.muS;

end