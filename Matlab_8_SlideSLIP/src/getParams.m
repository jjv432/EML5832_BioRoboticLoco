function params = getParams()

    % Parameters
    params.l0 = 1;  %* m
    params.min_l = params.l0*.4;
    params.b = 1500;  %*
    params.k = 10000; %*
    params.g = 9.81;
    params.m = 80;  %*
    params.phi_0 = -pi/4;  %*
    params.muS = 1; 
    params.muK = (.7)*params.muS;
    params.t_hip = 5000;
    params.stance_bool = 1;

    zeta = params.b/(2*sqrt(params.m*params.k));


end