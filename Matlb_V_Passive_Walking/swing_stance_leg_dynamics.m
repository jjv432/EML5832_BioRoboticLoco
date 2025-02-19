function func = swing_stance_leg_dynamics(t, y, gamma)

            % g = 9.81;
            g = 1;
            L = 1;

            theta = y(1);
            theta_d = y(2);
            phi = y(3);
            phi_d = y(4);

            theta_dd = (g/L)*sin(theta - gamma);
            phi_dd = theta_dd + (theta_d^2)*sin(phi) - (g/L)*cos(theta-gamma)*sin(phi);

            func = [theta_d; theta_dd; phi_d; phi_dd];

            if (phi - 2*theta) == 0
                tempy3 = y(3);
                tempy4 = y(4);

                y(3) = y(1);
                y(4) = y(2);

                y(1) = tempy3;
                y(2) = tempy4;
            end


        end