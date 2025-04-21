function params = getParams()

    % Parameters
    params.l0 = 1;  %* m
    % params.b = 1500;  %*
    params.b = 750;  %*
    params.k = 20000; %*
    params.g = 9.81;
    params.m = 80;  %*
    params.phi_0 = -pi/4.5;  %*
    params.muS = 1; 
    params.muK = (.5)*params.muS;

    zeta = params.b/(2*sqrt(params.m*params.k));


end