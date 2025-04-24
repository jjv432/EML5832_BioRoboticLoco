function func = slide_dynamics(t,y, params)

    % Values that won't change
    persistent b m k l0 g muK min_l
    if isempty(b)
        b = params.b;
        k = params.k;
        m = params.m;
        l0 = params.l0;
        g = params.g;
        muK = params.muK;
        min_l = params.min_l;
    end
    t_hip = params.t_hip;

    % Parse out the states
    l = y(1);
    l_d = y(2);
    phi = y(3);
    phi_d = y(4);
    s = y(5);
    s_d = y(6);

    if l < params.min_l
        l = min_l;
        l_d = 0;
    end

    % Adding sliding dynamics
    Fleg = k*(l0 - l) - b*l_d;
    Fleg_x = Fleg*sin(phi);
    Ff_leg = (m*g + Fleg*cos(phi))*muK*sin(phi);
    Ff_x = (m*g + Fleg*cos(phi))*muK*cos(phi);
    l_dd = l*phi_d^2 -g*cos(phi) - Fleg/m - Ff_leg/m;

    t_fric = (m*g + Fleg*cos(phi))*muK*l*cos(phi);
    phi_dd = (1/l)*(-2*l_d*phi_d) + (1/l)*(g*sin(phi)) + t_hip/(m*l*l) + t_fric/(m*l*l);

    % Making sure directions of forces are perserved; lazy fix
    % Not sure if this is flipped
    Fleg_x = abs(Fleg_x);
    Ff_x = abs(Ff_x);
    if phi < 0
        s_dd = (Fleg_x - Ff_x)/m;
    elseif phi > 0
        s_dd = (Ff_x - Fleg_x)/m;
    else
        s_dd = 0;
    end

    func = [l_d; l_dd; phi_d; phi_dd; s_d; s_dd];

end
