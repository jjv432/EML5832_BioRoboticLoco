function [position,isterminal,direction] = slide_event_func(t,y, params)

    % Values that won't change
    persistent k l0 b t_hip
    if isempty(k)
        k = params.k;
        l0 = params.l0;
        b = params.b;
        t_hip = params.t_hip;
    end
    
    % Parse out the values
    l = y(1);
    l_d = y(2);
    phi = y(3);
    phi_d = y(4);
    x = y(5);
    x_d = y(6);

    Fleg = k*(l0 - l) - b*l_d;

    position(1) = Fleg*cos(phi) + (1/l)*t_hip*sin(phi);
    isterminal(1) = 1;
    direction(1) = -1;
    % 
    % % Transition to flight
    % position(1) = l - l0;
    % isterminal(1) = 1;
    % direction(1) = 1;

    % Transition to stance
    % Fleg = k*(l0 - l) - b*l_d;
    % 
    persistent muK
    if isempty(muK)
        muK = params.muK;
    end
    % 
    % Fleg_x = abs(Fleg*sin(phi));
    % Fleg_y = abs(Fleg*cos(phi));
    % Ffk = Fleg_y * muK;

    % position(2) = Ffk - Fleg_x;
    position(2) = tan(phi) - muK;
    isterminal(2) = 1;
    direction(2) = -1;

end

