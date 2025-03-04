classdef Quadrotor < handle
    %QUADROTOR Summary of this class goes here
    %   Detailed explanation goes here
    
    properties

        % Indices
        formationIndex        % k
        quadrotorIndex        % i
        totalNum

        % Parameters
        quadrotorParameters   % mass_ki; armlength_ki; I_xx_ki; I_yy_ki; I_zz_ki ; g
        noiseMean
        noiseStd

        % Passivity Indices
        nu
        rho
        
        % Local Controller Gains
        localControllerGains1 = []
        localControllerGains2 = []

        % Global ControllerGains
        controllerGains1 = []
        controllerGains2 = []
        
        % Data to be distributed (in decentralized schemes)
        dataToBeDistributed
        controllerGainsCollection

        % States
        noise                 % disturbances (dx, dv, dR, dOmega)
        desiredSeparation     % From the leader
        desiredSeparations    % From all others
        % desiredAcceleration   % Leader's acceleration
        Omegad                % Desired angular velocity
        Rd
        % Re
        C
        C_dot
        X
        fd
        R_i
        controlInput
        states                % [quadrotorPosition; quadrotorTransVelocity; quadrotorAttitude(:,1); quadrotorAttitude(:,2);
                              %  quadrotorAttitude(:,3); quadrotorAngVelocity] (position, velocity, orientation (b1, b2, b3), angular velocity)

        % Control inputs
        leaderControlInput
        followerControlInput  

        % Errors
        errors                % e_xi, e_vi, e_ai, e_Omegai
        errors_bar            % errors_bar = T_i*errors
        e_Omega
        % outputs

        % state history
        followerStateHistory = []
        leaderStateHistory = []

        % error history
        errorHistory = []

        % Predefined controls
        plannedLeaderStates = []
        plannedControls = []  % matrix of pairs [t_i, u_i]

        % GeometricProperties (for plotting)          
        inNeighbors = []
        outNeighbors = []

        % graphicHandles
        graphics = []

    end
    
    methods

        function obj = Quadrotor(k,i,parameters,states,desiredSeparation,noiseMean,noiseStd,totalNumOfQuadrotors)

            % Constructor
            obj.formationIndex = k;
            obj.quadrotorIndex = i;
            obj.totalNum = totalNumOfQuadrotors;

            obj.quadrotorParameters = parameters;                 % [mass_ki; armlength_ki; J_xx_ki; J_yy_ki; J_zz_ki ; g; varepsilon_i]

            obj.states = states;                                  % states of the i^{th} quadrotor [quadrotorPosition; quadrotorTransVelocity; quadrotorAttitude(:,1); quadrotorAttitude(:,2); quadrotorAttitude(:,3); quadrotorAngVelocity]
            obj.Omegad = zeros(3,1);
            obj.fd = zeros(3,1);
            obj.controlInput = zeros(6,1);
            obj.Rd = eye(3);
            obj.C = zeros(3);
            obj.C_dot = zeros(3);
            obj.X = zeros(3,1);

            obj.desiredSeparation = desiredSeparation;            % need to satisfy this constraint (ith quadrotor's desired separation from the leader)
            
            % External disturbances represented by random noise
            obj.noiseMean = noiseMean;                      
            obj.noiseStd = noiseStd;                        

            obj.noise = [zeros(3,1); ...
                noiseMean + noiseStd.*randn(3,1); ...
                zeros(3,1); ...
                noiseMean + noiseStd.*randn(3,1)];          
            
            % Initial system values
            obj.errors = zeros(12,1);
            obj.errors_bar = zeros(6,1);
            obj.e_Omega = zeros(3,1);
            obj.leaderControlInput = zeros(3,1);
            obj.followerControlInput = zeros(6,1); % [parameters(1)*parameters(6); zeros(3,1)];        % the followers' control input (transformed quadrotor tracking error dynamics input)
            % obj.outputs = zeros(12,1);        

            % obj.desiredAcceleration = zeros(3,1);          

            obj.inNeighbors = [];
            obj.outNeighbors = [];
            
        end

        
        %% This function is used to draw a "quadrotor" shape object (Draw the initial quadrotors)
        function outputArg = drawQuadrotor(obj,figNum)
            figure(figNum); hold on;
            
            if ~isempty(obj.graphics)
                delete(obj.graphics);
            end
            
            % Collect all parameters for quadrotor drawing
            armlength = obj.quadrotorParameters(2);
            
            % Set initial orientation for quadrotor drawing
            R = [obj.states(7:9)'; obj.states(10:12)'; obj.states(13:15)'];
            b2w = [R obj.states(1:3); 0 0 0 1];
            quadBody = [armlength, 0, 0, 1; ...
                    0, -armlength,    0, 1; ...
               -armlength,      0,    0, 1; ...
                    0,  armlength,    0, 1; ...
                    0,      0,     0, 1; ...
                    0,      0, -0.15, 1]';
            quadWorld = b2w * quadBody;      % [4x4][4x6]
            quadPosition = quadWorld(1:3, :); 

            % Draw quadrotor arms (1-3, 2-4, and onboard payload)
            obj.graphics(1) = plot3(gca, quadPosition(1,[1 3]), quadPosition(2,[1 3]), quadPosition(3,[1 3]), ...
                    '-ro', 'MarkerSize', 5, 'Linewidth', 3);     % arm 1-3 x-y-z
            obj.graphics(2) = plot3(gca, quadPosition(1,[2 4]), quadPosition(2,[2 4]), quadPosition(3,[2 4]), ...
                    '-bo', 'MarkerSize', 5, 'Linewidth', 3);     % arm 2-4 x,y,z
            obj.graphics(3) = plot3(gca, quadPosition(1,[5 6]), quadPosition(2,[5 6]), quadPosition(3,[5 6]), ...
                    '-k', 'Linewidth', 3);
            
            % Coordinate of the quadrotor numbers updated with real time quadrotor position
            pos = obj.states(1:3) + [0; 0; -1];
            
            % Show quadrotors' number
            obj.graphics(4) = text(pos(1),pos(2),pos(3),[num2str(obj.quadrotorIndex-1)],'FontSize',12); 
            
        end



        %% This function is used to redraw (update) the quadrotors in real time
        function outputArg = redrawQuadrotor(obj,figNum)
            figure(figNum); hold on;

            if ~isempty(obj.graphics)
                delete(obj.graphics);
            end
            
            % Collect all parameters for quadrotor drawing
            armlength = obj.quadrotorParameters(2);
            
            R = [obj.states(7:9)'; obj.states(10:12)'; obj.states(13:15)'];
            b2w = [R obj.states(1:3); 0 0 0 1];
            quadBody = [armlength, 0, 0, 1; ...
                    0, -armlength,     0, 1; ...
               -armlength,      0,     0, 1; ...
                    0,  armlength,     0, 1; ...
                    0,      0,     0, 1; ...
                    0,      0, -0.15, 1]';
            quadWorld = b2w * quadBody;      % [4x4][4x6]
            quadPosition = quadWorld(1:3, :); 

            % Draw quadrotor arms (1-3, 2-4, and onboard payload)
            obj.graphics(1) = plot3(gca, quadPosition(1,[1 3]), quadPosition(2,[1 3]), quadPosition(3,[1 3]), ...
                    '-ro', 'MarkerSize', 5, 'Linewidth', 3);
            obj.graphics(2) = plot3(gca, quadPosition(1,[2 4]), quadPosition(2,[2 4]), quadPosition(3,[2 4]), ...
                    '-bo', 'MarkerSize', 5, 'Linewidth', 3);
            obj.graphics(3) = plot3(gca, quadPosition(1,[5 6]), quadPosition(2,[5 6]), quadPosition(3,[5 6]), ...
                    '-k', 'Linewidth', 3);
            
            % Coordinate of the quadrotor numbers updated with real time quadrotor position
            pos = obj.states(1:3) + [0; 0; -1];
            
            % Quadrotor number
            obj.graphics(4) = text(pos(1),pos(2),pos(3),[num2str(obj.quadrotorIndex - 1)],'FontSize',12);
    
        end
        


        %% Generate the external disturbances (noise)
        function outputArg = generateNoise(obj)
            if obj.quadrotorIndex==1
                w = zeros(3,1);      % Leader is not affected by the noise.
            else 
                w = [zeros(3,1); ...
                obj.noiseMean + obj.noiseStd.*randn(3,1); ...
                zeros(3,1); ...
                obj.noiseMean + obj.noiseStd.*randn(3,1)];
            end

            obj.noise = w;
        end


        %% Compute the quadrotor's errors
        function outputArg = computeQuadrotorErrors2Part1(obj, leaderStates, t, tMax)
            separationFromLeader = obj.desiredSeparation; 
            
            if t == 0
                newPositionErrors = obj.states(1:3) - leaderStates(1:3) + [separationFromLeader;0;0];  % position tracking errors update                                   
                newTransVelocityErrors = obj.states(4:6) - leaderStates(4:6);   % (body) velocity tracking errors update
                
                obj.errors(1:3) = newPositionErrors; 
                obj.errors(4:6) = newTransVelocityErrors;
                obj.errors_bar = obj.errors(1:6);

            elseif (t > 0) && (t < 0.5*tMax)
                newPositionErrors = obj.states(1:3) - leaderStates(1:3) + [separationFromLeader;0;0];    % position tracking errors update
                newTransVelocityErrors = obj.states(4:6) - leaderStates(4:6);   % (body) velocity tracking errors update
                
                obj.errors(1:3) = newPositionErrors; 
                obj.errors(4:6) = newTransVelocityErrors;
                obj.errors_bar = obj.errors(1:6);
            
            elseif t >= 0.5*tMax
                newPositionErrors = obj.states(1:3) - leaderStates(1:3) + [separationFromLeader;0;0]; % [0;separationFromLeader;0];   % position tracking errors update                
                newTransVelocityErrors = obj.states(4:6) - leaderStates(4:6);   % (body) velocity tracking errors update
                
                obj.errors(1:3) = newPositionErrors; 
                obj.errors(4:6) = newTransVelocityErrors;
                obj.errors_bar = obj.errors(1:6);

            end
        end


        function outputArg = computeQuadrotorErrors2Part2(obj, leaderStates, dt, t)
            
            I = eye(3);
            O = zeros(3);
            R = [obj.states(7:9) obj.states(10:12) obj.states(13:15)];
            varepsilon = obj.quadrotorParameters(7);
            e3 = [0 0 1]';

            if t == 0    
                
                bd3 = -(obj.fd)/norm(obj.fd);
                b1 = leaderStates(4:6)/norm(leaderStates(4:6));
                bd1 = -(((hatmap(bd3))^2)*b1)./norm(((hatmap(bd3))^2)*b1);
                bd2 = (hatmap(bd3)*b1)./norm(hatmap(bd3)*b1);
                obj.Rd = [bd1 bd2 bd3];
                
                Re = obj.Rd'*R;
                % Re_dot = (obj.Rd*hatmap(obj.Omegad))'*R+obj.Rd'*(R*hatmap(obj.states(16:18)));
                obj.X = norm(obj.fd)*((e3'*obj.Rd'*R*e3)*R*e3-obj.Rd*e3);     

                newAttitudeErrors = 0.5*veemap(Re-Re');                        % orientation tracking errors update
                newAngVelocityErrors = obj.states(16:18)-Re'*obj.Omegad;       % angular velocity tracking errors update
                
                obj.errors(7:9) = newAttitudeErrors; 
                obj.errors(10:12) = newAngVelocityErrors;

                % obj.C = 0.5*(trace(Re')*I-Re');
                % obj.C_dot = 0.5*(trace(Re_dot')*I-Re_dot');
                % T = [I  O  O  O;
                %      O  I  O  O;
                %      O  O  1/varepsilon*I O;
                %      O  O  O  obj.C];        % transformation matrix T
      
                % Transformed errors
                % obj.errors_bar = T*obj.errors;
                obj.errorHistory = [obj.errorHistory; obj.errors'];

            else 
                Rd_Old = obj.Rd;     % store the old Rd
                                
                bd3 = -(obj.fd)/norm(obj.fd);
                b1 = leaderStates(4:6)/norm(leaderStates(4:6));
                bd1 = -(((hatmap(bd3))^2)*b1)./norm(((hatmap(bd3))^2)*b1);
                bd2 = (hatmap(bd3)*b1)./norm(hatmap(bd3)*b1);
                obj.Rd = [bd1 bd2 bd3];      % Compute the new Rd
                
                Re = obj.Rd'*R;
                obj.X = norm(obj.fd)*((e3'*Re*e3)*R*e3-obj.Rd*e3);

                newAttitudeErrors = 0.5*veemap(Re-Re');        % orientation tracking errors update   

                obj.Omegad = 1/(2*dt)*veemap(Rd_Old'*obj.Rd-obj.Rd'*Rd_Old);
                % Re_dot = (obj.Rd*hatmap(obj.Omegad))'*R+obj.Rd'*(R*hatmap(obj.states(16:18)));
                
                % obj.C = 0.5*(trace(Re')*I-Re');
                % obj.C_dot = 0.5*(trace(Re_dot')*I-Re_dot');
                % T = [I  O  O  O;
                %      O  I  O  O;
                %      O  O  1/varepsilon*I  O;
                %      O  O  O  obj.C];        % transformation matrix T

                newAngVelocityErrors = obj.states(16:18)-Re'*obj.Omegad;    % angular velocity tracking errors update
                
                % Transformed errors
                obj.errors(7:9) = newAttitudeErrors; 
                obj.errors(10:12) = newAngVelocityErrors;
                % obj.errors_bar = T*obj.errors;
                obj.errorHistory = [obj.errorHistory; obj.errors'];

            end

        end


        %% Compute the quadrotor's control inputs
        function outputArg = computeControlInputs2Part1(obj, t, neighborInformation)
            
            if obj.quadrotorIndex==1  % Leader's control (from planned)
                
                if obj.plannedControls(1,1)==t
                    obj.leaderControlInput = obj.plannedControls(1,2:4)';  
                    obj.plannedControls = obj.plannedControls(2:end,:);   % Delete the executed planned control   
                else
                    obj.leaderControlInput = zeros(3,1);
                end

            else                      % Followers control (based on errors) under Error-Dynamics - II
                
                i = obj.quadrotorIndex;
                % O = zeros(3,1);
                L_ii = obj.controllerGains2{i} + obj.localControllerGains2;   % This parameter is \bar{L}_{ii}+L_{ii}\in\mathbb{R}^{3\times 6}
                % e_xvi_bar = [obj.errors_bar(1:6); O; O];        % Note that e_{xi} = \bar{e}_xi, e_{vi}=\bar{e}_vi, and e_xvi_bar = [e_{xi};e_{vi};O;O]
                e_i_bar = obj.errors_bar;

                % obj.controlInput(1:3) = L_ii(1:3,:)*e_xvi_bar;
                obj.controlInput(1:3) = L_ii*e_i_bar;
                for jInd = 1:1:length(obj.inNeighbors)
                    j = obj.inNeighbors(jInd);
                    if j~=1
                        L_ij = obj.controllerGains2{j};         % L_ij
                        % e_xvj_bar = neighborInformation{j};     % Use the shared neighboring information e_{xi} and e_{vi} (e_{xi} = \bar{e}_xi, e_{vi}=\bar{e}_vi)
                        % obj.controlInput(1:3) = obj.controlInput(1:3) + L_ij(1:3,:)*(e_xvi_bar - e_xvj_bar);
                        e_j_bar = neighborInformation{j};
                        obj.controlInput(1:3) = obj.controlInput(1:3) + L_ij*(e_i_bar - e_j_bar);
                    end 
                end
                
                e3 = [0 0 1]';
                obj.fd = obj.quadrotorParameters(1)*(obj.controlInput(1:3)-obj.quadrotorParameters(6)*e3);
                obj.followerControlInput(1:3) = obj.fd;

            end
        end


        function outputArg = computeControlInputs2Part2(obj)
            
            i = obj.quadrotorIndex;

            if i > 1 % Followers control (based on errors) under Error-Dynamics - II

                % O = zeros(3,1);
                I = eye(3);
                % L_ii = obj.controllerGains2{i} + obj.localControllerGains2;   % \bar{L}_{ii}+L_{ii}
                % e_ROmegai_bar = [O; O; obj.errors_bar(7:12)];   % Note that e_{Ri} = varepsilon*\bar{e}_{Ri}, e_{Omegai}=(Ci^{-1})*\bar{e}_{Omegai}, and e_ROmegai_bar = Ti*[O;O;e_{Ri};e_{Omegai}]
                e_ROmegai = obj.errors(7:12);

                %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
                % l11 = 20;
                % l12 = 20;
                % l13 = 0;
                % l14 = 0;
                % l21 = 0;
                % l22 = 0;
                l23 = 5/(0.1);
                l24 = 5/0.1;   % We add some perturbation parameter here

                LiiBar = -[ l23*I l24*I ];

                % obj.controlInput(4:6) = L_ii(4:6,:)*e_ROmegai_bar;
                obj.controlInput(4:6) = LiiBar*e_ROmegai;
                %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
                % for jInd = 1:1:length(obj.inNeighbors)
                %     j = obj.inNeighbors(jInd);
                %     if j~=1
                %         L_ij = obj.controllerGains2{j};         % L_ij
                %         e_ROmegaj_bar = neighborInformation{j};       % Use the shared neighboring information e_{xi} and e_{vi} (e_{xi} = \bar{e}_xi, e_{vi}=\bar{e}_vi)
                %         obj.controlInput(4:6) = obj.controlInput(4:6) + L_ij(4:6,:)*(e_ROmegai_bar - e_ROmegaj_bar);
                %     end 
                % end

                J = diag(obj.quadrotorParameters(3:5));
                % obj.followerControlInput(4:6) = J*inv(obj.C)*(obj.controlInput(4:6)-obj.C_dot*inv(obj.C)*obj.errors_bar(10:12));
                          % the controller to update e_Omega (no last two terms involved)
                obj.followerControlInput(4:6) = J*(obj.controlInput(4:6));
                
            end
        end


        %% Update the state values of the system dynamics
        % Update the leader's and followers' dynamics
        function quadrotorError = dynamicsUpdate(obj, t, dt, tMax)
            
            % Here, we simply collect the errors computed above and return it back to the Formation layer to further compute the overall errors.
            quadrotorError = obj.errors;

            i = obj.quadrotorIndex;

            if i == 1  % Leader's states update (by planned control)
                
                % Leader's states updates
                if t < 0.5*tMax
                    obj.states(4:6) = obj.plannedLeaderStates(1,:)';
                elseif t < tMax
                    obj.states(4:6) = obj.plannedLeaderStates(2,:)';
                else
                    obj.states(4:6) = obj.plannedLeaderStates(3,:)';
                end
                leaderPositionUpdate = obj.states(4:6); 
                leaderTransVelocityUpdate = obj.leaderControlInput;

                newLeaderPosition = obj.states(1:3) + dt * leaderPositionUpdate;   % discretization method?
                newLeaderVelocity = obj.states(4:6) + dt * leaderTransVelocityUpdate;
                obj.states(1:3) = newLeaderPosition;
                obj.states(4:6) = newLeaderVelocity;
                
                % Collect all the states and desired acceleration at each time instant
                obj.leaderStateHistory = [obj.leaderStateHistory; obj.states']; 

            else       % Followers states update
                
                % Define some necessary matrices and vectors
                e3 = [0 0 1]';
                J = diag(obj.quadrotorParameters(3:5));
                R = [obj.states(7:9) obj.states(10:12) obj.states(13:15)];
                
                % Followers' states updates 
                followerPositionUpdate = obj.states(4:6);                                 % Position update
                % obj.followerControlInput = [obj.quadrotorParameters(1)*obj.quadrotorParameters(6)-1; zeros(3,1)];
                % obj.noise = zeros(12,1);
                obj.X = norm(obj.fd)*((e3'*obj.Rd'*R*e3)*R*e3-obj.Rd*e3);
                followerTransVelovityUpdate = 1/obj.quadrotorParameters(1)*(obj.followerControlInput(1:3)-obj.X)+ ...
                        obj.quadrotorParameters(6)*e3+1/obj.quadrotorParameters(1)*obj.noise(4:6);  % Translational velocity update
                followerAngVelovityErrorUpdate = inv(J)*(obj.followerControlInput(4:6)+obj.noise(10:12));    
                                   % We update the augular velocity error (body frame) using M'= M-J*Omegad_dot instead of the angular velocity directly
                
                % New followers' states
                newPosition = obj.states(1:3) + dt * followerPositionUpdate;              % New position
                newTransVelovity = obj.states(4:6) + dt * followerTransVelovityUpdate;    % New translational velocity
                newR = update_rotation_matrix(R,obj.states(16:18),dt);                    % New R (The update of orientation is different to other dynamics)
                
                newfollowerAngVelovityError = obj.errors(10:12) + dt * followerAngVelovityErrorUpdate;
                newAngVelocity = newfollowerAngVelovityError + R'*obj.Rd*obj.Omegad;  % New angular velocity (in body frame: Omega = followerAngVelovityError+Omegad)

                newStates = [newPosition;newTransVelovity;newR(:,1);newR(:,2);newR(:,3);newAngVelocity];
                obj.states = newStates;                     
                

                % Collect all the state points at each step
                obj.followerStateHistory = [obj.followerStateHistory; obj.states'];
            end

        end



        %% Local Control Synthesis
        % Load passivity properties \rho_i and nu_i
        function outputArg = loadPassivityIndices(obj,nu,rho)
            obj.nu = nu;
            obj.rho = rho;
        end



        
        

        %% Parametrized Approach to Synthesize Local Controllers (while also assisting synthesis of global controllers)
        function [status,PVal,KVal,LVal,nuVal,rhoVal,gammaSqVal] = synthesizeLocalControllersParameterized(obj,errorDynamicsType,pVal)
            % Here we will synthesize the local controllers for local error
            % dynamics to optimize the passivity properties
            % This is the true local controller synthesis for a given p_i value
            
            I = eye(3);
            O = zeros(3);
            I_n = eye(6);
            O_n = zeros(6);
            % Ones = ones(3);
            % I_n = eye(12);
            % O_n = zeros(12);
            % O_K = zeros(6,12);
            % O_K = zeros(3,6);            
            
            % varepsilon = obj.quadrotorParameters(7);

            if errorDynamicsType == 1
                A = [O, I, O, O;
                     O, O, O, O;
                     O, O, O, I;
                     O, O, O, O];    % For error dynamics type 1
            else
                % A = [O, I, O, O;
                %      O, O, O, O;
                %      O, O, O, 1/varepsilon*I;
                %      O, O, O, O];    % For error dynamics type 2  
                A = [O, I; O, O];    % For error dynamics type 2  
            end
            
            % B = [O, O;
            %      I, O;
            %      O, O;
            %      O, 1/varepsilon*I];
            
            B = [O; I];
            
            % Set up the LMI problem
            solverOptions = sdpsettings('solver','mosek','verbose',0);            
            % P = sdpvar(12,12,'symmetric'); 
            % K = sdpvar(6,12,'full'); 

            P = sdpvar(6,6,'symmetric'); 
            K = sdpvar(3,6,'full');
        
            nu = sdpvar(1,1,'full');
            rhoTilde = sdpvar(1,1,'full'); % Representing: 1/rho
            gammaSq = sdpvar(1,1,'full');
            
            % For: nuBar < nu < nuHat < 0
            nuBar = -gammaSq/pVal;
        
            % For: 0 < rhoHat1,rhoHat2 < rho < rhoBar
            % For: 0 < rhoTildeHat < rhoTilde < rhoTildeBar1,rhoTildeBar2
            rhoTildeBar1 = 4*gammaSq/pVal;
            rhoTildeBar2 = pVal;
        
            % Basic Constraints
            con1 = P >= 0;
            
            % Approach 4 with rho = prespecified, nu < 0 and nu is free to maximize
            DMat = [rhoTilde*I_n];
            MMat = [P, O_n];
            ThetaMat = [-A*P-P*A'-B*K-K'*B', -I_n+0.5*P; 
                        -I_n+0.5*P, -nu*I_n];
            W = [DMat, MMat; 
                 MMat', ThetaMat];
            con2 = W >= 0;
            
            %%Constraints on resulting nu and rho from the local design 
            % nuBar < nu < nuHat < 0 
            con3 = nu >= nuBar;              % Helps global design
            
            % 0 < rhoTildeHat < rhoTilde < rhoTildeBar1,rhoTildeBar2  
            con4 = rhoTilde <= rhoTildeBar1;    % Helps global design
            con5 = rhoTilde <= rhoTildeBar2;    % Helps global design

            %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
            % We constrain the format of the P, \bar{L}_{ii}, and \tilde{L}_{ii}
            % nullMatxP = [ O    O   Ones Ones;
            %               O    O   Ones Ones;
            %              Ones Ones  O    O;
            %              Ones Ones  O    O];
            % nullMatxK = [ O    O   Ones Ones;
            %              Ones Ones  O    O];

            % con6 = P.*(nullMatxP==1) == O_n;
            % con7 = K.*(nullMatxK==1) == O_K;
            %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
            
            % Total Cost and Constraints
            cons = [con1,con2,con3,con4,con5];
            % cons = [con1,con2,con3,con4,con5,con6,con7];
            %costFun =  0*(-nu + rhoBar);           % For stabilizing, set coefficient to 0
            %costFun = 0.0000001*(-nu + rhoBar);    % Otherwise set to 0.0000001.
            costFun = 0*gammaSq;
        
            % Solution
            sol = optimize(cons,costFun,solverOptions);
            status = sol.problem == 0;   % sol.info;
        
            PVal = value(P);     % This is the P_i value obtained from solving the local control synthesis
            KVal = value(K);     % This is \tilde{L}_{ii}
            LVal = KVal/PVal;    % This LVal is the local controller gain \bar{L}_{ii}

            %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
            % We need to involve the singular perturbation (varepsilon) here for local controller gain \bar{L}_{ii}!
            % LVal(4:6,:) = 1/varepsilon*LVal(4:6,:);
            %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
            
            RVal = inv(PVal);
            obj.R_i = RVal;

            nuVal = value(nu);
            rhoVal = 1/value(rhoTilde);
            gammaSqVal = value(gammaSq);
        
            % Updating the information
            obj.nu = nuVal;
            obj.rho = rhoVal;

            obj.localControllerGains1 = LVal;     % Here we need \bar{k}_{i0}^{Local} = 1
            obj.localControllerGains2 = LVal; 
            
        end


        %% Decentralized Robust Controller Synthesis With sMS Constraints (Error Dynamics II)
        function [isRobustStabilizable,K_ii,K_ijVals,K_jiVals,gammaSq_iVal,statusL,LVal] = robustControllerSynthesissMS2(obj, previousSubsystems, subsystems, pVal, displayMasseges, isSoft)

            % i = length(previousSubsystems)+1;
            iInd = obj.quadrotorIndex-1;
            N = iInd;
            costCoefficient1 = 1;
            costCoefficient2 = 1; 
            costCoefficient3 = 1; 
            varepsilon = obj.quadrotorParameters(7);   % Singular perturbation parameter
            
            % Add PVal and KVal in the original local controller synthesis function here to be used later
            [statusL,PVal,KVal,LVal,nuVal,rhoVal,gammaSqLVal] = obj.synthesizeLocalControllersParameterized(2, pVal);
            if displayMasseges
                % disp(['Robust Stabilizing at: ',num2str(iInd),' after ',num2str(previousSubsystems),'.']);
                if statusL == 1
                    % disp(['Local Synthesis Success at: ',num2str(iInd),' with gammaSq=',num2str(value(gammaSqLVal)),'.'])
                else
                    disp(['Local Synthesis Failed at: ',num2str(iInd),'.']);
                end
            end

            if statusL == 0
                isRobustStabilizable = 0;
                return
            end 

            % Setting up the LMI problem
            solverOptions = sdpsettings('solver','mosek','verbose',0);
            % isSoft = 1;
            % normType = 2;  % Type of the norm to be used in the LMI
            minCostVal = 0.01;

            % I_n = eye(12);       
            % O_n = zeros(12);
            I_n = eye(6);       
            O_n = zeros(6);
            Ones = ones(3);
            O = zeros(3);

            nu_i = obj.nu;
            rho_i = obj.rho;
            X_i_11 = -nu_i*I_n;
            X_i_12 = 0.5*I_n;
            X_i_21 = X_i_12';
            X_i_22 = -rho_i*I_n;
            X_ii_12 = X_i_11\X_i_12;
            X_ii_21 = X_ii_12';
            
            % Set a fixed value for epsilon_i and make epsilon_i+rho_i>1 satisfied
            epsilon_i = 1.01-rho_i;  

            % min and max eigenvalues of R_i
            RVal = inv(PVal);
            obj.R_i = RVal;
            MaxEigR_i = max(eig(RVal));
            MinEigR_i = min(eig(RVal));

            obj.dataToBeDistributed.X = X_ii_12;    

            % null_ii = [ Ones Ones  Ones Ones;
            %              O    O    Ones Ones;
            %             Ones Ones  Ones Ones;
            %             Ones Ones   O    O];
            % cost_ii = 0*[O    O    O    O;
            %             Ones Ones Ones Ones;
            %              O    O    O    O;
            %             Ones Ones Ones Ones];

            null_ii = [ Ones Ones;
                         O    O ];
            cost_ii = 0*[ O    O;
                         Ones Ones ];
            
            if isempty(previousSubsystems)

                % The subsystem only need to test W_ii > 0 (since in this case, there are no subsystems before this ith subsystem)
                p_i = sdpvar(1,1);
                % Q_ii = sdpvar(12,12,'full'); 
                Q_ii = sdpvar(6,6,'full');
                gammaSq_i = sdpvar(1,1);   % This gammaSq_i is the \hat{gamma}_i in decentralized co-design

                costFun0 = sum(sum(Q_ii.*cost_ii));
                % costFun0 = norm(Q_ii.*cost_ii,1);

                con1 = p_i >= 0;

                DMat_ii = [p_i*X_i_11, O_n; 
                           O_n, I_n];
                MMat_ii = [Q_ii, p_i*X_i_11;
                           I_n, O_n]; 
                TMat_ii = [-X_ii_21*Q_ii-Q_ii'*X_ii_12-p_i*X_i_22, -p_i*X_i_21; 
                           -p_i*X_i_12, gammaSq_i*I_n];

                W_ii = [DMat_ii, MMat_ii; 
                        MMat_ii', TMat_ii];     

                con2 = W_ii >= 0; %10^(-6)*eye(size(W_ii));

                con3 = Q_ii.*(null_ii==1) == O_n;     
                
                %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
                % Additional DSS constraint 1 (on K_ii) is added here (for the first follower, there are no neighboring vehicles)
                con4 = RVal*Q_ii+Q_ii'*RVal-p_i*nu_i*epsilon_i*I_n <= 0;
                %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

                costFun =  costCoefficient1*costFun0 + costCoefficient2*gammaSq_i + costCoefficient3*(gammaSq_i-gammaSqLVal)^2;

                % Our modified optimization problem with the DSS constraint (con4)
                sol = optimize([con1,con2,con3,con4],costFun,solverOptions);
                isRobustStabilizable = sol.problem == 0;

                p_iVal = value(p_i);
                Q_iiVal = value(Q_ii);
                gammaSq_iVal = value(gammaSq_i);
                % costFun0Val = value(costFun0);

                con2Val = value(W_ii);
                eigVals = eig(con2Val);

                W_iiVal = value(W_ii);
                tildeW_i = W_iiVal; % Note that, here, \tilde{W}_ii = W_ii = \tilde{W}_i. This also needs to be stored

                K_ii = (p_iVal*X_i_11)\Q_iiVal;
                % K_ii(10:12,:) = 1/varepsilon*K_ii(10:12,:);
                obj.controllerGainsCollection.decenRobustCont1{iInd} = K_ii;

                K_jiVals = [];
                K_ijVals = [];

                obj.dataToBeDistributed.P = p_iVal;
                obj.dataToBeDistributed.gammaSq = gammaSq_iVal;
                obj.dataToBeDistributed.tildeW = tildeW_i;  % Storing

                if displayMasseges                                            
                    disp(['Data saved at: ',num2str(iInd),'.']);
                    if ~isRobustStabilizable
                        disp(['LMI is not feasible at: ',num2str(iInd),', thus terminating here.']);
                    else
                        disp(['LMI is feasible at: ',num2str(iInd),', thus continued.'])
                    end
                end

                if ~isRobustStabilizable && (min(eigVals)<-0.000001 || p_iVal < 0)
                    disp(['Decentralized Synthesis Failed at: ',num2str(iInd),'.'])
                else
                    isRobustStabilizable = 1;
                    % disp(['Decentralized Synthesis Success at: ',num2str(iInd),' with gammaSq=',num2str(value(gammaSq_iVal)),'.'])
                end

            else
                % This subsystem has to talk with all the previousSubsystems
                % tildeW_ii > 0 iff [M_i, W_i'; W_i, W_ii] > 0 is required where 
                % M_i = inv((scriptD_i*scriptA_i^T)^{-1}*(scriptD_i)*(scriptD_i*scriptA_i^T)^{-1}') = scriptA_i*scriptD_i*scriptA_i'

                % M_i term
                blockSize = 24;     
                for j = 1:1:length(previousSubsystems)
                    jInd = previousSubsystems(j);

                    % Getting stored info from jInd to create \mathcal{A}_i and \mathcal{D}_i matrices (their j-th columns)
                    tildeW_j = subsystems(jInd+1).dataToBeDistributed.tildeW;

                    if j == 1
                        tildeW_jj = tildeW_j;
                        M_i = tildeW_jj;
                    else
                        tildeW_jj = tildeW_j(:,blockSize*(j-1)+1:blockSize*j);   % last blockSizeXblockSize block in the row block vector
                        tildeW_j  = tildeW_j(:,1:blockSize*(j-1));               % first (j-1) blocks in the row block vector
                        M_i = [M_i, tildeW_j'; tildeW_j, tildeW_jj];             % This tildeW_jj is iteratively updated based on the previous stored tildeW_ii
                    end                    
                end

                % LMI variables
                p_i = sdpvar(1,1);
                % Q_ii = sdpvar(12,12,'full');
                Q_ii = sdpvar(6,6,'full');
                gammaSq_i = sdpvar(1,1);

                % W_ii and W_i terms
%                 DMat = [X_p_11, O; O, I];
%                 MMat = [Q, X_p_11; I, O];
%                 ThetaMat = [-X_21*Q-Q'*X_12-X_p_22, -X_p_21; -X_p_12, gammaSq*I];
%                 con4 = [DMat, MMat; MMat', ThetaMat] >= 0; % The real one  
 
                % When i = j  
                DMat_ii = [p_i*X_i_11, O_n; 
                           O_n, I_n];
                MMat_ii = [Q_ii, p_i*X_i_11; 
                           I_n, O_n]; 
                TMat_ii = [-X_ii_21*Q_ii-Q_ii'*X_ii_12-p_i*X_i_22, -p_i*X_i_21; 
                           -p_i*X_i_12, gammaSq_i*I_n];
                W_ii = [DMat_ii, MMat_ii; 
                        MMat_ii', TMat_ii];

                W_i = [];                            
                for j = 1:1:length(previousSubsystems)

                    jInd = previousSubsystems(j);

                    % null_ij{j} = [Ones Ones  Ones Ones;
                    %                O    O    Ones Ones;
                    %               Ones Ones  Ones Ones;
                    %               Ones Ones   O    O];
                    % null_ji{j} = [Ones Ones  Ones Ones;
                    %                O    O    Ones Ones;
                    %               Ones Ones  Ones Ones;
                    %               Ones Ones   O    O];

                    null_ij{j} = [ Ones  Ones;
                                    O     O ];
                    null_ji{j} = [ Ones  Ones;
                                    O     O ];
                    
                    %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
                    % In order to solve the optimization with sMS constraint, 
                    % we confine the structure of Q_ij and Q_ji using these matrices, where only the last element has value

                    % null_ij_sMS{j} = [1 1 1;1 1 1;1 1 0]; 
                    % null_ji_sMS{j} = [1 1 1;1 1 1;1 1 0];
                    %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

                    if any(obj.inNeighbors==jInd) % iInd <----- jInd
                        % adj_ij{j} = [O    O    O    O;
                        %             Ones Ones  O    O;
                        %              O    O    O    O;
                        %              O    O   Ones Ones]; % is in-neighbors
                        % cost_ij{j} = 1*[O    O    O    O;
                        %                Ones Ones  O    O;
                        %                 O    O    O    O;
                        %                 O    O   Ones Ones];
                        adj_ij{j} = [ O    O;
                                     Ones Ones ]; % is in-neighbors
                        cost_ij{j} = 1*[ O    O;
                                        Ones Ones ];
                    else
                        % adj_ij{j} = [O, O, O, O;
                        %              O, O, O, O;
                        %              O, O, O, O;
                        %              O, O, O, O]; % not in-neighbors
                        % cost_ij{j} = (20/N)*abs(iInd-jInd)*[O    O    O    O;
                        %                                    Ones Ones  O    O;
                        %                                     O    O    O    O;
                        %                                     O    O   Ones Ones];
                        adj_ij{j} = [ O, O;
                                      O, O ]; % not in-neighbors
                        cost_ij{j} = (20/N)*abs(iInd-jInd)*[ O    O;
                                                            Ones Ones ];
                    end

                    if any(obj.outNeighbors==jInd) % iInd -----> jInd
                        % adj_ji{j} = [O    O    O    O;
                        %             Ones Ones  O    O;
                        %              O    O    O    O;
                        %              O    O   Ones Ones]; % is out-neighbors
                        % cost_ji{j} = 1*[O    O    O    O;
                        %                Ones Ones  O    O;
                        %                 O    O    O    O;
                        %                 O    O   Ones Ones];
                        adj_ji{j} = [ O     O;
                                     Ones  Ones ]; % is out-neighbors
                        cost_ji{j} = 1*[ O    O;
                                        Ones Ones ];

                    else
                        % adj_ji{j} = [O, O, O, O;
                        %              O, O, O, O;
                        %              O, O, O, O;
                        %              O, O, O, O];    % not out-neighbors
                        % cost_ji{j} = (20/N)*abs(iInd-jInd)*[O    O    O    O;
                        %                                    Ones Ones  O    O;
                        %                                     O    O    O    O;
                        %                                     O    O   Ones Ones];
                        adj_ji{j} = [ O, O;
                                      O, O ];    % not out-neighbors
                        cost_ji{j} = (20/N)*abs(iInd-jInd)*[ O    O;
                                                            Ones Ones ];
                    end

                    % Q_ij{j} = sdpvar(12,12,'full');
                    % Q_ji{j} = sdpvar(12,12,'full');
                    Q_ij{j} = sdpvar(6,6,'full');
                    Q_ji{j} = sdpvar(6,6,'full');

                    X_jj_12 = subsystems(jInd+1).dataToBeDistributed.X;

                    % DMat = [X_p_11, O; O, I];
                    % MMat = [Q, X_p_11; I, O];
                    % ThetaMat = [-X_21*Q-Q'*X_12-X_p_22, -X_p_21; -X_p_12, gammaSq*I];
                    % con4 = [DMat, MMat; MMat', ThetaMat] >= 0;   % The real one  
                    DMat_ij = [O_n, O_n; 
                               O_n, O_n];
                    MMat_ij = [Q_ij{j}, O_n; 
                               O_n, O_n];
                    MMat_ji = [Q_ji{j}, O_n; 
                               O_n, O_n];
                    TMat_ij = [-X_ii_21*Q_ij{j}-Q_ji{j}'*X_jj_12, O_n;
                               O_n, O_n];

                    % When i \neq j
                    W_ij = [DMat_ij, MMat_ij; 
                            MMat_ji', TMat_ij];
                    W_i = [W_i, W_ij];

                end

                con1 = p_i >= 0;

                % con2Mat = [M_i, W_i';W_i, W_ii];
                con2 = [M_i, W_i';
                        W_i, W_ii] >= 0;     %10^(-6)*eye(size(M_i)+size(W_ii));

                con3 = []; % Soft graph constraints
                con3_hard = []; % Hard graph constraints
                costFun0 = 0;
                con5_value = 0;

                % To Guarantee sMS, we confine the K_ij (Q_ij) values with our proposed alternative DSS condition (DSS constraint 2 option 1)
                % (for the second and other followers, the information from neighboring vehicles are considered, i.e., K_ij)
                for j = 1:1:length(previousSubsystems)

                    % jInd = previousSubsystems(j);
                    % Confine the Q_{ij} and Q_{ji} values with the unique structure
                    con3_ij = Q_ij{j}.*(null_ij{j}==1) == O_n;
                    con3_ji = Q_ji{j}.*(null_ji{j}==1) == O_n;
                    con3 = [con3, con3_ij, con3_ji];

                    con3_ij_hard = Q_ij{j}.*(adj_ij{j}==0) == O_n;
                    con3_ji_hard = Q_ji{j}.*(adj_ji{j}==0) == O_n;
                    con3_hard = [con3_hard, con3_ij_hard, con3_ji_hard]; 

                    costFun0 = costFun0 + sum(sum(Q_ij{j}.*cost_ij{j})) + sum(sum(Q_ji{j}.*cost_ji{j}));
                    % costFun0 = costFun0 + norm(Q_ij{j}.*cost_ij{j},1) + norm(Q_ji{j}.*cost_ji{j},1);
                    
                    %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
                    % Iteratively add the K_ij values to the LHS of the DSS constraint (DSS constraint 2 option 1)
                    mu_i = (rho_i+epsilon_i-1)/MaxEigR_i;
                    con5_value = con5_value + sqrt(1/(mu_i*MinEigR_i))*norm(RVal*Q_ij{j})+1/(2^j)*(p_i*nu_i);  
                    %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
                end
                
                % Confine the Q_{ii} value with the unique structure
                con3_ii = Q_ii.*(null_ii==1) == O_n;

                % Collect all the constraints on Q_{ij}'s value
                con3 = [con3, con3_ii];
                
                %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
                % The additional sMS constraint 1 (on Q_{ii} or K_{ii}) is added here 
                con4 = RVal*Q_ii+Q_ii'*RVal-p_i*nu_i*epsilon_i*I_n <= 0;
                
                % To Guarantee DSS, we confine the K_ij (Q_ij) values with our proposed alternative DSS condition (DSS constraint 2 option 2)
                % (for the second and other followers, the information from neighboring vehicles are considered, i.e., K_ij)
                 
                % for j = 1:1:length(previousSubsystems)
                %     con3_ij = Q_ij{j}.*(null_ij_DSS{j}==1) == O_n;
                %     con3_ji = Q_ji{j}.*(null_ji_DSS{j}==1) == O_n;
                %     con3 = [con3, con3_ij, con3_ji];
                % 
                %     con3_ij_hard = Q_ij{j}.*(adj_ij{j}==0) == O_n;
                %     con3_ji_hard = Q_ji{j}.*(adj_ji{j}==0) == O_n;
                %     con3_hard = [con3_hard, con3_ij_hard, con3_ji_hard]; 
                % 
                %     costFun0 = costFun0 + sum(sum(Q_ij{j}.*cost_ij{j})) + sum(sum(Q_ji{j}.*cost_ji{j}));
                % 
                %     %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
                %     % Iteratively add the K_ij values to the LHS of the DSS constraint
                % 
                %     mu_i = (rho_i+epsilon_i-1)/MaxEigR_i;
                %     con5_value = con5_value + sqrt(1/(mu_i*MinEigR_i))*norm(R_i*Q_ij{j})+1/(2^j)*(p_i*nu_i);  
                %     %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
                % end
                % K_ij = k_ij*[0 0 0; 0 0 0; 0 0 1];

                % Remaining DSS constraints
                % First attempt: follow the expression in the paper
                % Second attempt: use a different norm, i.e., 1-norm
                % Third attempt: confine ourselves to a specific type of K_ij value, say a scalar k_ij
                con5 = con5_value <= 0;

                % These two conditions are not necessary if epsilon_i is carefully selected 
                % con6 = rho_i+epsilon_i-1 > 0;
                % con7 = epsilon_i > 0;
                %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

                costFun0 = costFun0 + sum(sum(Q_ii.*cost_ii));
                % costFun0 = costFun0 + norm(Q_ii.*cost_ii,1);

                con0 = costFun0 >= minCostVal;
                % iInd*minCostVal/5;  %0.0001;
                
                if isSoft               
                    cons = [con0,con1,con2,con3,con4,con5];
                else
                    cons = [con0,con1,con2,con3,con3_hard,con4,con5];
                end

                costFun =  costCoefficient1*costFun0 + costCoefficient2*gammaSq_i + costCoefficient3*(gammaSq_i-gammaSqLVal)^2;
                
                % Here, we solve the co-design optimization problem with DSS constraints
                sol = optimize(cons,costFun,solverOptions);
                isRobustStabilizable = sol.problem==0;

                p_iVal = value(p_i);
                Q_iiVal = value(Q_ii);
                gammaSq_iVal = value(gammaSq_i);
                costFun0Val = value(costFun0);

                con2Val = value([M_i, W_i';W_i, W_ii]);
                eigVals = eig(con2Val);   

                W_iVal = value(W_i);
                W_iiVal = value(W_ii);
                
                % Obtain the solved K_{ii} values
                K_ii = (p_iVal*X_i_11)\Q_iiVal;
                % K_ii(10:12,:) = 1/varepsilon*K_ii(10:12,:);
                obj.controllerGainsCollection.decenStabCont1{iInd} = K_ii;

                for j = 1:1:length(previousSubsystems)
                    jInd = previousSubsystems(j);

                    Q_ijVal = value(Q_ij{j});
                    Q_jiVal = value(Q_ji{j});
                    p_jVal = subsystems(jInd+1).dataToBeDistributed.P;
                    X_j_11 = -subsystems(jInd+1).nu*I_n;
                    
                    % Obtain the solved K_{ij} values
                    K_ij = (p_iVal*X_i_11)\Q_ijVal;
                    % K_ij(10:12,:) = 1/varepsilon*K_ij(10:12,:);
                    K_ijVals{jInd} = K_ij;
                    obj.controllerGainsCollection.decenStabCont1{jInd} = K_ij;
                    
                    % Obtain the solved K_{ji} values
                    K_ji = (p_jVal*X_j_11)\Q_jiVal;
                    % K_ji(10:12,:) = 1/varepsilon*K_ji(10:12,:);
                    K_jiVals{jInd} = K_ji; % these values will be loaded to subsystem j outside of this function
                end

                % Need to compute \tilede{W}_i and \tilde{W}_{ii} for storage
                tildeW_i = W_iVal;
                tildeW_ii = W_iiVal;

                tildeW_i = [tildeW_i, tildeW_ii];

                % Storing
                obj.dataToBeDistributed.P = p_iVal;             
                obj.dataToBeDistributed.gammaSq = gammaSq_iVal;
                obj.dataToBeDistributed.tildeW = tildeW_i;      

                if displayMasseges
                    disp(['Data saved at: ',num2str(iInd),' after ',num2str(previousSubsystems),'.']);
                    if ~isRobustStabilizable
                        disp(['LMI is not feasible at: ',num2str(iInd),' after ',num2str(previousSubsystems),'.']);
                    else
                        disp(['LMI is feasible at: ',num2str(iInd),' after ',num2str(previousSubsystems),'.']);
                    end
                end

                if ~isRobustStabilizable && (min(eigVals)<-0.000001 || p_iVal < 0)
                    disp(['Decentralized Synthesis Failed at: ',num2str(iInd),'.'])
                else
                    isRobustStabilizable = 1;
                    % disp(['Decentralized Synthesis Success at: ',num2str(iInd),' with gammaSq=',num2str(value(gammaSq_iVal)),'.'])
                end 

            end

        end
        


    end
end
