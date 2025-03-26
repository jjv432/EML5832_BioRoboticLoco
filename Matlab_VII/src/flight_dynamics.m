function func = flight_dynamics(t, y, params)

    persistent g
    if isempty(g)
        g = params.g;
    end

    x = y(1);
    x_d = y(2);
    z = y(3);
    z_d = y(4);

    x_dd = 0;
    z_dd = -g;

    func = [x_d; x_dd; z_d; z_dd];

end