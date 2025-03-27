function [position,isterminal,direction] = walker_event(t,y)
    position = y(3) - 2*y(1); % The value that we want to be zero
    isterminal = 1;  % Halt integration
    direction = 1;   
end