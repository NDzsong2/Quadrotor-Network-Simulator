classdef Quadrotor < handle
    %QUADROTOR Summary of this class goes here
    %   Detailed explanation goes here
    
    properties

        % Indices
        formationIndex        % k
        quadrotorIndex        % i

        % Parameters
        quadrotorParameters   % mass_ki; armlength_ki; I_xx_ki; I_yy_ki; I_zz_ki
        noiseMean
        noiseStd

        % % Passivity Indices
        % nu
        % rho
        % 
        % % Local Controller Gains
        % localControllerGains1 = []
        % localControllerGains2 = []

        % % Global ControllerGains
        % controllerGains1 = []
        % controllerGains2 = []
        
        % Data to be distributed (in decentralized schemes)
        dataToBeDistributed
        controllerGainsCollection

        % States
        desiredSeparation     %From the leader
        desiredSeparations    %From all others
        states                % x_ik
        noise                 % disturbances (dv, dw)
        controlInput
        errors
        outputs

        % state history
        stateHistory = []

        % error history
        errorHistory = []

        % Predefined controls
        plannedControls = []  % matrix of paris [t_i,u_i]

        % GeometricProperties (for plotting)          
        inNeighbors = []
        outNeighbors = []

        % graphicHandles
        graphics = []

    end
    
    methods

        function obj = Quadrotor(k,i,parameters,states,desiredSeparation,noiseMean,noiseStd)

            % Constructor
            obj.formationIndex = k;
            obj.quadrotorIndex = i;

            obj.quadrotorParameters = parameters;                 % [mass,length1,length2,smallBar1,smallBar2,smallBar3,smallBar4,propRadius1,propRadius2,propRadius3,propRadius4]

            obj.states = states;                                  % states of the i^{th} quadrotor (x,v,R,w)
            obj.desiredSeparation = desiredSeparation;            % need to track this signal (desired position,velocity and 0 acceleration for i^{th} quadrotor)
            
            
            % External disturbances represented by random noise
            obj.noiseMean = noiseMean;
            obj.noiseStd = noiseStd;

            obj.noise = [zeros(3,1); ...
                noiseMean + noiseStd.*randn(3,1); ...
                zeros(3,1); ...
                noiseMean + noiseStd.*randn(3,1)];
            
            %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
            % Initial system values
            obj.errors = zeros(12,1);
            obj.controlInput = zeros(4);
            obj.outputs = zeros(12,1);
            %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
            
            obj.inNeighbors = [];
            obj.outNeighbors = [];
            
        end

        
        % This function is used to draw a "quadrotor" shape object
        function outputArg = drawQuadrotor(obj,figNum)
            figure(figNum); hold on;
            
            if ~isempty(obj.graphics)
                delete(obj.graphics);
            end
            
            % Collect all parameters for quadrotor drawing
            armlength = obj.quadrotorParameters(2);
            % smallBar1 = obj.quadrotorParameters(2)/8;
            % smallBar2 = obj.quadrotorParameters(2)/8;
            % smallBar3 = obj.quadrotorParameters(2)/8;
            % smallBar4 = obj.quadrotorParameters(2)/8;
            % propRadius1 = obj.quadrotorParameters(2)/3;
            % propRadius2 = obj.quadrotorParameters(2)/3;
            % propRadius3 = obj.quadrotorParameters(2)/3;
            % propRadius4 = obj.quadrotorParameters(2)/3;

            R0 = eye(3);
            wHb = [R0 obj.states(1:3); 0 0 0 1];
            quadBody = [armlength, 0, 0, 1; ...
                    0, -armlength,     0, 1; ...
               -armlength,      0,     0, 1; ...
                    0,  armlength,     0, 1; ...
                    0,      0,     0, 1; ...
                    0,      0, -0.15, 1]';
            quadWorld = wHb * quadBody; % [4x4][4x6]
            quadPosition = quadWorld(1:3, :); 

            % Draw quadrotor arms (1-3, 2-4, payload)
            obj.graphics(1) = plot3(gca, quadPosition(1,[1 3]), quadPosition(2,[1 3]), quadPosition(3,[1 3]), ...
                    '-ro', 'MarkerSize', 5, 'Linewidth', 3);
            obj.graphics(2) = plot3(gca, quadPosition(1,[2 4]), quadPosition(2,[2 4]), quadPosition(3,[2 4]), ...
                    '-bo', 'MarkerSize', 5, 'Linewidth', 3);
            obj.graphics(3) = plot3(gca, quadPosition(1,[5 6]), quadPosition(2,[5 6]), quadPosition(3,[5 6]), ...
                    '-k', 'Linewidth', 3);
            
            % Real time quadrotor position
            % pos = obj.states(1:3);
            
           
            % % Draw the quadrotor body (two arms, four short bars and four propellers)
            % poly1 = polyshape([pos, (pos-length), (pos-length), pos],[0.5, 0.5, 0.5+height1, 0.5+height1]);
            % obj.graphics(1) = plot(poly1,'FaceColor','k');
            % 
            % poly2 = polyshape([0.5*(pos+0.5*(pos+(pos-length))), 0.5*((pos-length)+0.5*(pos+(pos-length))),...
            %     0.5*((pos-length)+0.5*(pos+(pos-length))), 0.5*(pos+0.5*(pos+(pos-length)))],...
            %     [0.5+height1, 0.5+height1, (0.5+height1)+height2, (0.5+height1)+height2]);
            % obj.graphics(2) = plot(poly2,'FaceColor','k');
            % 
            % % Draw four short bars where the propellers are installed on
            % obj.graphics(3) = plot(poly2,'Color','k');
            % obj.graphics(4) = plot(poly2,'Color','k');
            % obj.graphics(5) = plot(poly2,'Color','k');
            % obj.graphics(6) = plot(poly2,'Color','k');
            % 
            % % Draw four propellers
            % obj.graphics(7) = viscircles([0.5*((pos-length)+0.5*(pos+(pos-length))), 0.3], propRadius1,'Color','k');
            % obj.graphics(8) = viscircles([0.5*(pos+0.5*(pos+(pos-length))), 0.3], propRadius2,'Color','k');
            % obj.graphics(9) = viscircles([0.5*(pos+0.5*(pos+(pos-length))), 0.3], propRadius3,'Color','k');
            % obj.graphics(10) = viscircles([0.5*(pos+0.5*(pos+(pos-length))), 0.3], propRadius4,'Color','k');
            % 
            % 
            % % Quadrotor number
            % obj.graphics(11) = text(pos,0.2,num2str(obj.quadrotorIndex));
            
        end



        % This function is used to plot the real time states (i.e., position) of 
        % the i^{th} quadrotor at the position before each quadrotor
        function outputArg = redrawQuadrotor(obj,figNum)
            figure(figNum); hold on;

            if ~isempty(obj.graphics)
                delete(obj.graphics);

                length1 = obj.quadrotorParameters(2);
                length2 = obj.quadrotorParameters(2);
                height1 = obj.quadrotorParameters(2)/8;
                height2 = obj.quadrotorParameters(2)/8;
                radius = obj.quadrotorParameters(2)/16;
                pos = obj.states(1);
            
           
                % Draw the quadrotor body
                poly1 = polyshape([pos, (pos-length), (pos-length), pos],[0.5, 0.5, 0.5+height1, 0.5+height1]);
                obj.graphics(1) = plot(poly1,'FaceColor','r');
                
                poly2 = polyshape([0.5*(pos+0.5*(pos+(pos-length))), 0.5*((pos-length)+0.5*(pos+(pos-length))),...
                0.5*((pos-length)+0.5*(pos+(pos-length))), 0.5*(pos+0.5*(pos+(pos-length)))],...
                [0.5+height1, 0.5+height1, (0.5+height1)+height2, (0.5+height1)+height2]);
                obj.graphics(2) = plot(poly2,'FaceColor','r');
            
                % Draw four propellers
                obj.graphics(3) = viscircles([0.5*((pos-length)+0.5*(pos+(pos-length))), 0.3], radius,'Color','k');
                obj.graphics(4) = viscircles([0.5*(pos+0.5*(pos+(pos-length))), 0.3], radius,'Color','k');
                
                % Quadrotor number
                obj.graphics(5) = text(pos,0,num2str(obj.quadrotorIndex));
            end
            
        end



        % Update the state values of the system dynamics
        function vehicleError = update(obj,t,dt)
            
            vehicleError = obj.errors;

            A = [0 1 0; 0 0 1; 0 0 0];
            B = [0 0 1]';

            updateValue = A*obj.states + B*obj.controlInput + obj.noise;
            
            newStates = obj.states + dt*(updateValue);
            obj.states = newStates;                     
            
            % Collect all the state points at each step
            obj.stateHistory = [obj.stateHistory, newStates];

        end


    end
end
