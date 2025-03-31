function [position,isterminal,direction] = flight_event_func(t,y, params)

    % Assuming that the leg 'snaps' to l = l0, phi = phi0
    % Values that won't change:
    persistent phi0 l0
    if isempty(phi0)
        phi0 = params.phi_0;
        l0 = params.l0;
    end

    z = y(3);

    position(1) = l0*cos(phi0) - z;
    isterminal(1) = 1;  % Halt integration
    direction(1) = 1;   % Zero approached by decreasing values

    % This is an event that is used to determine the apex height. Was only
    % useful for the newton-raphson
    position(2) = y(4);
    isterminal(2) = 0;  % Halt integration
    direction(2) = -1;   % Zero approached by decreasing values

end