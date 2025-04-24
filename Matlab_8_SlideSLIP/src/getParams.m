function params = getParams()

    % Parameters
    params.l0 = 1;  %* m
    params.min_l = params.l0*.4;
    params.b = 2500;  %*
    params.k = 10000; %*
    params.g = 9.81;
    params.m = 80;  %*
    params.phi_0 = -pi/3.5;  %*
    params.muS = 3; 
    params.muK = (.25)*params.muS;

    zeta = params.b/(2*sqrt(params.m*params.k));


end