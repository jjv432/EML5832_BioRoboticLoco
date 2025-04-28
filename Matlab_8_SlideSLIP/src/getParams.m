function params = getParams()

    % Parameters
    params.l0 = 1;  %* m
    params.min_l = params.l0*.4;
    params.b = 2500;  %*
    params.k = 10000; %*
    params.g = 9.81;
    params.m = 80;  %*
    params.phi_0 = -pi/7;  %*
    params.muS = 100; 
    params.muK = (.9)*params.muS;
    params.t_hip = 30;
    params.stance_bool = 1;

    zeta = params.b/(2*sqrt(params.m*params.k));


end