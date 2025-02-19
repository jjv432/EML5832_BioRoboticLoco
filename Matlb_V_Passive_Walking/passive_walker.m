classdef passive_walker < handle
    %UNTITLED2 Summary of this class goes here
    %   Detailed explanation goes here

    properties
        init_state (4, 1)
        T
        Y
        leg_1_coords
        leg_2_coords
        gamma
    end

    methods
        function obj = passive_walker(init_state)
            %UNTITLED2 Construct an instance of this class
            %   Detailed explanation goes here
            obj.init_state(1) = init_state(1);
            obj.init_state(2) = init_state(2);
            obj.init_state(3) = init_state(3);
            obj.init_state(4) =  init_state(4);
        end

        function GenerateDynamics(obj, time, gamma)
            %METHOD1 Summary of this method goes here
            %   Detailed explanation goes here

            obj.gamma = gamma;
            [obj.T, obj.Y] = ode45(@(T, Y) swing_stance_leg_dynamics(T, Y, gamma), time, obj.init_state);

        end

        function CreateCoordinates(obj, index)

            theta = obj.Y(index, 1);
            phi = obj.Y(index, 3);
            adjustment = pi + obj.gamma;
            template_coords = [0 .1 .1 0; 0 0 1 1];
           
            rot_matrix_1 = [cos(theta + adjustment), -sin(theta + adjustment); sin(theta + adjustment), cos(theta + adjustment)];
            rot_matrix_2 = [cos(phi + adjustment), -sin(phi + adjustment); sin(phi + adjustment), cos(phi + adjustment)];

            obj.leg_1_coords = rot_matrix_1 * template_coords;
            obj.leg_2_coords = rot_matrix_2 * template_coords;

        end

        function DrawWalker(obj, index)
            obj.CreateCoordinates(index);

            ax = gca;
            axis equal
            patch(obj.leg_1_coords(1, :), obj.leg_1_coords(2, :), 'r', 'Parent', ax);
            patch(obj.leg_2_coords(1, :), obj.leg_2_coords(2, :), 'b', 'Parent', ax);

        end

        function AnimateWalker(obj)
            figure()
            axis([-1 1 -1 1])

            for i = 1:numel(obj.T)
                obj.CreateCoordinates(i);
                obj.DrawWalker(i)
                pause(.1);
                cla
            end
        end
                

        
    end
end