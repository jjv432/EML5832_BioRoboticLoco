function params = getParams()

    % Parameters
    params.l0 = 1;  %* m
    params.b = 15;  %*
    params.k = 105; %*
    params.g = 9.81;
    params.m = 1;  %*
    params.t_hip = 2.9;  %*
    params.phi_0 = -pi/5.5;  %*
    params.phi_d_0 = 1.5;
    params.l_d_0 = -4;
    params.muS = .3; % Good for sliding
    params.muK = (.5)*params.muS;

end