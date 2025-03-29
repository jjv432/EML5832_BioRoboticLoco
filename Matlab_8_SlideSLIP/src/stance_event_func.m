function [position,isterminal,direction] = stance_event_func(t,y, params)

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

    position = Fleg*cos(phi) + (1/l)*t_hip*sin(phi);
    isterminal = 1;
    direction = -1;
end