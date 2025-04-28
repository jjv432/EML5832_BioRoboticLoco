function [position,isterminal,direction] = slide_event_func(t,y, params)

    % Values that won't change
    persistent k l0 b t_hip muK stance_bool
    if isempty(k)
        k = params.k;
        l0 = params.l0;
        b = params.b;
        t_hip = params.t_hip;
        muK = params.muK;
    end

    % Parse out the values
    l = y(1);
    l_d = y(2);
    phi = y(3);
    phi_d = y(4);
    x = y(5);
    x_d = y(6);

    if isempty(stance_bool)
        stance_bool = 1;
    elseif (abs(tan(phi)) < muK)
        stance_bool = stance_bool -1;
    end

    Fleg = k*(l0 - l) - b*l_d;

    % Transisition to flight
    position(1) = Fleg*cos(phi) + (1/l)*t_hip*sin(phi);
    isterminal(1) = 1;
    direction(1) = -1;

    % Transistion to stance
    position(2) = stance_bool;
    isterminal(2) = 1;
    direction(2) = -1;


end

