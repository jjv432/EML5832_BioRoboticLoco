function params = getParams()

    % Parameters
    params.l0 = 1;  %* m
    params.b = 10;  %*
    params.k = 50; %*
    params.g = 9.81;
    params.m = 5;  %*
    params.t_hip = 7;  %*
    params.phi_0 = -pi/4.5;  %*
    % params.phi_d_0 = 1.5;
    % params.l_d_0 = -4;
    params.muS = .1; % Good for sliding
    params.muK = (.5)*params.muS;

end