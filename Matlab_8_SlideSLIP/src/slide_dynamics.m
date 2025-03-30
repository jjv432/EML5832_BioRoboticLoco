function func = slide_dynamics(t,y, params)

    persistent b m k l0 g t_hip muK
    if isempty(b)
        b = params.b;
        k = params.k;
        m = params.m;
        l0 = params.l0;
        g = params.g;
        t_hip = params.t_hip;
        muK = params.muK;
    end

    l = y(1);
    l_d = y(2);
    phi = y(3);
    phi_d = y(4);
    x = y(5);
    x_d = y(6);

    % Adding sliding dynamics
    Fleg = k*(l0 - l) - b*l_d;
    Fleg_x = abs(Fleg*sin(phi));
    Fleg_y = Fleg*cos(phi);
    Ffk = abs(Fleg_y * muK);

    l_dd = l*phi_d^2 - g*cos(phi) - (k/m)*(l - l0) - (b/m)*l_d;
    phi_dd = (1/l)*(-2*l_d*phi_d) + (1/l)*(g*sin(phi)) + t_hip/(m*l^2) - (Fleg/m + Fleg*cos(phi)*sin(phi)*muK/m);

    % Making sure directions of forces are perserved; lazy fix
    if phi < 0
        Ffk = -Ffk;
    elseif phi >= 0
        Fleg_x = -Fleg_x;
    end

    x_dd = (Fleg_x - Ffk)/m;

    func = [l_d; l_dd; phi_d; phi_dd; x_d; x_dd];

end
