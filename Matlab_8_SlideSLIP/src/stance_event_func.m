function [position,isterminal,direction] = stance_event_func(t,y, params)

    % Event for a transition to flight

    persistent k l0 b t_hip
    if isempty(k)
        k = params.k;
        l0 = params.l0;
        b = params.b;
        t_hip = params.t_hip;
    end

    l = y(1);
    l_d = y(2);
    phi = y(3);
    phi_d = y(4);

    Fleg = k*(l0 - l) - b*l_d;

    % Transition to flight
    position(1) = l - l0;
    isterminal(1) = 1;
    direction(1) = 1;

    % Event for a transistion to sliding
    
    persistent muS
    if isempty(muS)
        muS = params.muS;
    end

    % Fleg_x = abs(Fleg*sin(phi));
    % Fleg_y = abs(Fleg*cos(phi));
    % Ffs = Fleg_y * muS;

    % position(2) = Fleg_x - Ffs;
    position(2) = tan(phi) - muS;
    isterminal(2) = 1;
    direction(2) = 1;


end