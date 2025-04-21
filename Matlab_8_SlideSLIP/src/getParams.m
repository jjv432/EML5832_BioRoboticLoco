function params = getParams()

    % Parameters
    params.l0 = 1;  %* m
    params.b = 800;  %*
    params.k = 20000; %*
    params.g = 9.81;
    params.m = 80;  %*
    params.t_hip = 0;  %*
    params.phi_0 = -pi/4.5;  %*
    params.muS = .1; % Good for sliding
    params.muK = (.5)*params.muS;

end