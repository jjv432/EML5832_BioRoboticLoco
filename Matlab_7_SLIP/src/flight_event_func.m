function [position,isterminal,direction] = flight_event_func(t,y, params)

    % Assuming that the leg 'snaps' to l = l0, phi = phi0
    persistent phi0 l0
    if isempty(phi0)
        phi0 = params.phi_0;
        l0 = params.l0;
    end

    z = y(3);

    position = l0*cos(phi0) - z;
    isterminal = 1;  % Halt integration
    direction = 1;   % Zero approached by decreasing values
end