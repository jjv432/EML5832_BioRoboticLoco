function func = stance_dynamics(t,y, params)

    persistent b m k l0 g t_hip
    if isempty(b)
        b = params.b;
        k = params.k;
        m = params.m;
        l0 = params.l0;
        g = params.g;
        t_hip = params.t_hip;
    end

    l = y(1);
    l_d = y(2);
    phi = y(3);
    phi_d = y(4);

    l_dd = l*phi_d^2 - g*cos(phi) - (k/m)*(l - l0) - (b/m)*l_d;
    phi_dd = (1/l)*(-2*l_d*phi_d) + (1/l)*(g*sin(phi)) + t_hip/(m*l^2);

    func = [l_d; l_dd; phi_d; phi_dd];

end
