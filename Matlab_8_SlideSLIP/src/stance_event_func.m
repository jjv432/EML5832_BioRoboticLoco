function [position,isterminal,direction] = stance_event_func(t,y, params)

    % Event for a transition to flight

    persistent k l0 b t_hip muS
    if isempty(k)
        k = params.k;
        l0 = params.l0;
        b = params.b;
        t_hip = params.t_hip;
        muS = params.muS;
    end

    l = y(1);
    l_d = y(2);
    phi = y(3);
    phi_d = y(4);

    
    Fleg = k*(l0 - l) - b*l_d;

    % To flight
    position(1) = Fleg*cos(phi) + (1/l)*t_hip*sin(phi);
    isterminal(1) = 1;
    direction(1) = -1;

    % Event for a transistion to sliding
    position(2) = abs(tan(phi)) - muS;
    isterminal(2) = 1;
    direction(2) = 1;


end