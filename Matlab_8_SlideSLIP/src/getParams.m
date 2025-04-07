function params = getParams()

    % Parameters
    params.l0 = 1;  %* m
    params.b = 17;  %*
    params.b = 1;  %*
    params.k = 100; %*
    params.g = 9.81;
    params.m = 1;  %*
    params.t_hip = 12;  %*
    params.phi_0 = -pi/6;  %*
    params.phi_d_0 = 1.5;
    params.l_d_0 = -3.5;
    % params.l_d_0 = -1;
    params.muS = .55; % Good for sliding
    % params.muS = 5; % Good for sticking
    params.muK = (.5)*params.muS;

end