        %% Compute the tracking errors from the bottom class Quadrotor to this Formation layer
        % % In this method, we assume that each quadrotor has access to the leader.
        % function outputArg = computeFormationErrors1(obj, t)
        %     leaderStates = obj.quadrotors(1).states;          % A fully actuated quadrotor with desired states x_d, v_d (R_d, w_d) 
        %                                                       % got from the next layer (some values have been changed in Network layer)
        %     for i = 2:1:obj.numOfQuadrotors
        %         neighborInformation = [];
        %         for jInd = 1:1:length(obj.quadrotors(i).inNeighbors)   % Here, jInd is the size
        %             j = obj.quadrotors(i).inNeighbors(jInd);           % Here, jInd is the index of the inNeighbors array
        %             neighborInformation{j} = [obj.quadrotors(j).states; obj.quadrotors(j).desiredSeparation];  % 18+1 elements
        %         end 
        %         obj.quadrotors(i).computeQuadrotorErrors1(t, leaderStates, neighborInformation);
        %     end
        % 
        % end


        %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
        % In this method, we assume that each quadrotor has only access to the predecessor and its direct follower.
        % function outputArg = computeFormationErrors2(obj)
        %     leaderStates = obj.quadrotors(1).states;          % x_0,v_0,a_0
        %     for i = 2:1:obj.numOfQuadrotors
        %         neighborInformation = [];
        %         for jInd = 1:1:length(obj.quadrotors(i).inNeighbors)
        %             j = obj.quadrotors(i).inNeighbors(jInd);
        %             neighborInformation{j} = obj.quadrotors(j).states;
        %         end 
        %         obj.quadrotors(i).computeQuadrotorErrors2(leaderStates,neighborInformation);
        %     end
        % end
        %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

        %% Compute the control input from the bottom class Quadrotor to this Formation layer
        % % In this method, we assume that each quadrotor has access to the leader.
        % function outputArg = computeControlInputs1(obj, t)
        %     leaderStates = obj.quadrotors(1).states;
        %     for i = 2:1:obj.numOfQuadrotors
        %         neighborInformation = [];
        %         for jInd = 1:1:length(obj.quadrotors(i).inNeighbors)   % Here, jInd is the size
        %             j = obj.quadrotors(i).inNeighbors(jInd);           % Here, jInd is the index of the inNeighbors array
        %             neighborInformation{j} = [obj.quadrotors(j).states; obj.quadrotors(j).desiredSeparation];  % 18+1 elements, use index to place information
        %         end 
        %     end
        %     for i = 1:1:obj.numOfQuadrotors
        %         obj.quadrotors(i).computeControlInputs1(t, leaderStates, neighborInformation);
        %     end
        % end


        %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
        % In this method, we assume that each quadrotor has only access to the predecessor and the direct follower.
        % function outputArg = computeControlInputs2(obj, t)
        %     for i = 1:1:obj.numOfQuadrotors
        %         neighborInformation = [];
        %         for jInd = 1:1:length(obj.quadrotors(i).inNeighbors)
        %             j = obj.quadrotors(i).inNeighbors(jInd);
        %             neighborInformation{j} = obj.quadrotors(j).errors;
        %         end 
        %         obj.quadrotors(i).computeControlInputs2(t, neighborInformation);
        %     end
        % end
        %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

        % function outputArg = computeFormationErrors2(obj, t)
        %     leaderStates = obj.quadrotors(1).states;          % A fully actuated quadrotor with desired states x_d, v_d (R_d, w_d) 
        %                                                       % got from the next layer (some values have been changed in Network layer)
        %     for i = 2:1:obj.numOfQuadrotors
        %         neighborInformation = [];
        %         for jInd = 1:1:length(obj.quadrotors(i).inNeighbors)   % Here, jInd is the size
        %             j = obj.quadrotors(i).inNeighbors(jInd);           % Here, jInd is the index of the inNeighbors array
        %             neighborInformation{j} = [obj.quadrotors(j).states; obj.quadrotors(j).desiredSeparation];  % 18+1 elements
        %         end 
        %         obj.quadrotors(i).computeQuadrotorErrors2(t, leaderStates, neighborInformation);
        %     end
        % end


        % function outputArg = computeControlInputs2(obj, t)
        %     leaderStates = obj.quadrotors(1).states;
        %     for i = 2:1:obj.numOfQuadrotors
        %         neighborInformation = [];
        %         for jInd = 1:1:length(obj.quadrotors(i).inNeighbors)   % Here, jInd is the size
        %             j = obj.quadrotors(i).inNeighbors(jInd);           % Here, jInd is the index of the inNeighbors array
        %             neighborInformation{j} = [obj.quadrotors(j).states; obj.quadrotors(j).desiredSeparation];  % 18+1 elements, use index to place information
        %         end 
        %     end
        %     for i = 1:1:obj.numOfQuadrotors
        %         obj.quadrotors(i).computeControlInputs2(t, leaderStates, neighborInformation);
        %     end
        % end


%% Update the quadrotor tracking errors in real time
        % In this method, we assume that each quadrotor has access to the leader.
        % function outputArg = computeQuadrotorErrors1(obj, t, leaderStates, neighborInformation)
        % 
        %     % separationFromLeader = obj.desiredSeparation;
        %     i = obj.quadrotorIndex;
        % 
        %     if i > 1
        % 
        %         % Collect the neighbors of the ith quadrotor
        %         states_predecessor = neighborInformation{i-1};
        %         states_successor = neighborInformation{i+1};
        % 
        %         % e_x collection
        %         newPositionErrors = states_predecessor(1:3) - obj.states(1:3) - [states_successor(19)-states_predecessor(19); 0; 0];      %????????????????
        % 
        %         % e_v collection
        %         newTransVelocityErrors = -leaderStates(4:6) + obj.states(4:6);
        % 
        %         % e_R collection (done in the computeControlInputs1 method, since we have to know the direction of the desired thrust input)
        %         % Rd = [leaderStates(7:9)'; leaderStates(10:12)'; leaderStates(13:15)'];
        %         % Original orientation R
        %         R = [obj.states(7:9)'; obj.states(10:12)'; obj.states(13:15)'];
        %         Rd = obj.computeControlInputs1(t, leaderStates, neighborInformation); 
        %         Re = Rd' * R;
        %         newAttitudeErrors = 1/2 * veemap(Re - Re');
        % 
        %         % e_Omega collection
        % 
        % 
        %         S = R 
        %         Omega_d = 
        %         newAngVelocityErrors = obj.states(16:18) - R' * Rd * leaderStates(16:18);
        % 
        %         % All errors collection
        %         obj.errors = [newPositionErrors; newTransVelocityErrors; newAttitudeErrors(1,:)'; ...
        %                       newAttitudeErrors(2,:)'; newAttitudeErrors(3,:)'; newAngVelocityErrors];
        % 
        %         % Errors at each time instant
        %         obj.errorHistory = [obj.errorHistory, obj.errors];
        %     end
        % 
        % end
        

        %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
        % 先不做
        % In this method, we assume that each quadrotor has only access to the predecessor and the direct follower.
        % function outputArg = computeQuadrotorErrors2(obj,leaderStates,neighborInformation)
        % 
        %     locationError = 0;
        %     velocityError = 0;
        % 
        %     for jInd = 1:1:length(obj.inNeighbors)
        % 
        %         j = obj.inNeighbors(jInd);
        %         k_ijBar = obj.controllerGains1{j};
        %         d_ij = obj.desiredSeparations(j);
        %         X_j = neighborInformation{j};
        % 
        %         locationError_j = k_ijBar*(obj.states(1)-X_j(1)-d_ij);
        %         locationError = locationError + locationError_j;
        % 
        %         velocityError_j = k_ijBar*(obj.states(2)-X_j(2));
        %         velocityError = velocityError + velocityError_j;
        % 
        %     end
        % 
        %     accelerationError = obj.states(3)-leaderStates(3); %a_i-a_0
        % 
        %     newErrors = [locationError;velocityError;accelerationError];
        % 
        %     obj.errors = newErrors;
        %     obj.errorHistory = [obj.errorHistory, newErrors];
        % 
        % end
        %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
        

        % function computedRd = computeControlInputs1(obj, t, leaderStates, neighborInformation) 
        % 
        %     % oneUnderline = 1;
        %     S_2i = [0 0 0;
        %             0 0 0;
        %             0 0 1];
        % 
        %     F_gi = [0 0 obj.quadrotorParameters(1)*obj.quadrotorParameters(6)];
        % 
        %     i = obj.quadrotorIndex;
        % 
        %     if i == 1  % Leader's control (from planned)
        % 
        %         if obj.plannedControls(1,1) == t
        %             obj.leaderControlInput = obj.plannedControls(1, 2:4)';
        %             obj.plannedControls = obj.plannedControls(2:end, :); % Delete the executed planned control
        %         end
        % 
        %     elseif i < obj.totalNum                   % Followers control (based on errors) under Error-Dynamics - I
        % 
        %         % controller parameters
        %         K_Di = 100;
        %         K_Ri = 8.81;
        %         K_Omegai = 2.54;
        % 
        %         % Gather neighbors' information
        %         states_predecessor = neighborInformation{i-1};
        %         states_successor = neighborInformation{i+1};
        % 
        %         % Use the gathered neighbors' information, compute the fsi and fsi_next terms in the controllers
        %         fsi = fs(states_predecessor(1:3), obj.states(1:3), (obj.desiredSeparation-states_predecessor(19)));
        %         fsi_next = fs(obj.states(1:3), states_successor(1:3), (states_successor(19)-obj.desiredSeparation));
        % 
        % 
        %         % Use the computed fsi and fsi_next terms to compute the actual thrust controllers for i = 2,...,N-1 quadrotors
        %         obj.followerControlInput1 = obj.quadrotorParameters(1) * eye(3) * obj.desiredAcceleration - ...
        %                                    S_2i * F_gi' * 1 - K_Di * (obj.states(4:6) - leaderStates(4:6)) + ...
        %                                    fsi - fsi_next;
        % 
        %         % Compute the torque control input for i = 2,...,N-1
        %         % Rd = [leaderStates(7:9)'; leaderStates(10:12)'; leaderStates(13:15)'];
        %         % Only the first column of Rd is the leader's state, the next two columns are computed by the thrust force and the cross product 
        %         b1d = leaderStates(7:9);
        %         b3d = obj.followerControlInput1/norm(obj.followerControlInput1);
        %         b2d = hatmap(b3d) * b1d;
        % 
        %         % Return this computed Rd matrix for this method
        %         computedRd = [b1d, b2d, b3d];
        % 
        %         R = [obj.states(7:9)'; obj.states(10:12)'; obj.states(13:15)'];
        %         Re = Rd' * R;
        %         eR = 1/2 * veemap(Re - Re');
        % 
        %         eOmega = obj.states(16:18) - R' * Rd * leaderStates(16:18);
        %         J = diag([obj.quadrotorParameters(3), obj.quadrotorParameters(4), obj.quadrotorParameters(5)]);
        %         obj.followerControlInput234 = -K_Ri * eR - K_Omegai * eOmega + hatmap(obj.states(16:18)) * J * obj.states(16:18) - ...
        %                                      J * (hatmap(obj.states(16:18))*R'*Rd)
        % 
        % 
        %     else
        % 
        %         % Use the computed fsi to compute the actual controllers for i = N quadrotor
        %         states_predecessor = neighborInformation{i-1};
        %         fsi = fs(states_predecessor(1:3), obj.states(1:3), (obj.desiredSeparation-states_predecessor(19)));
        %         obj.followerControlInput1 = obj.quadrotorParameters(1) * eye(3) * obj.desiredAcceleration - ...
        %                                    S_2i * F_gi' * 1 - K_Di * (obj.states(4:6) - leaderStates(4:6)) + ...
        %                                    fsi;
        % 
        %         % Compute the torque control input for i = N
        % 
        %     end
        % end
        
        %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
        % 先不做
        % function outputArg = computeControlInputs2(obj, t, neighborInformation)
        % 
        %     if obj.quadrotorIndex == 1  % Leader's control (from planned)
        % 
        %         if obj.plannedControls(1,1) == t
        %             obj.leaderControlInput = obj.plannedControls(1,2);
        %             obj.plannedControls = obj.plannedControls(2:end,:); % Delete the executed planned control
        %         else
        %             obj.followerControlInput = 0;
        %         end
        % 
        %     else                    % Followers control (based on errors) under Error-Dynamics - II
        % 
        %         i = obj.quadrotorIndex;
        %         L_ii = obj.controllerGains2{i} + obj.localControllerGains2;
        %         e_i = obj.errors;
        %         controlInput = L_ii * e_i;
        %         for jInd = 1:1:length(obj.inNeighbors)
        %             j = obj.inNeighbors(jInd);
        %             if j~=1
        %                 L_ij = obj.controllerGains2{j};
        %                 e_j = neighborInformation{j};
        %                 controlInput = controlInput + L_ij*(e_i - e_j);
        %             end 
        %         end
        %         obj.followerControlInput = controlInput;
        % 
        %     end
        % end
        %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%




        % function quadrotorError = dynamicsUpdate(obj, t, dt)
        % 
        %     % Here, we simply collect the errors computed above and return it back to the Formation layer to further compute the overall errors.
        %     quadrotorError = obj.errors;
        % 
        %     i = obj.quadrotorIndex;
        % 
        %     if i == 1  % Leader's states (by planned control)
        % 
        %         % Leader's states updates
        %         leaderPositionUpdate = obj.states(4:6); 
        %         obj.desiredAcceleration = obj.leaderControlInput;
        % 
        %         newLeaderPosition = obj.states(1:3) + dt * leaderPositionUpdate;   % discretization method?
        %         obj.states(1:3) = newLeaderPosition;
        % 
        %         % Collect all the states and desired acceleration at each time instant
        %         obj.leaderStateHistory = [obj.leaderStateHistory; t', [obj.states; obj.desiredAcceleration]']; % 18+3 states in total at each time instant
        % 
        %     else                        % Followers update
        % 
        %       A = [0 1 0; 0 0 1; 0 0 0];
        %       B = [0 0 1]';
        % 
        %       updateValue = A * obj.states + B * obj.followerControlInput + obj.noise;
        % 
        %       newStates = obj.states + dt * (updateValue);
        %       obj.states = newStates;                     
        % 
        %       % Collect all the state points at each step
        %       obj.stateHistory = [obj.stateHistory, newStates];
        %     end
        % 
        % end


        % Update the state values of the leader's dynamics
        % function vehicleError = leaderUpdate(obj, t, dt)
        % 
        %     vehicleError = obj.errors;
        % 
        %     A = [0 0 0 1 0 0;...
        %          0 0 0 0 1 0;...
        %          0 0 0 0 0 1;...
        %          0 0 0 0 0 0;...
        %          0 0 0 0 0 0;...
        %          0 0 0 0 0 0];
        %     B = [0 0 0 1 0 0;...
        %          0 0 0 0 1 0;...
        %          0 0 0 0 0 1]';
        % 
        %     updateValue = A * obj.states(1:6) + B * obj.followerControlInput + obj.noise;
        % 
        %     newStates = obj.states + dt * (updateValue);
        %     obj.states = newStates;                     
        % 
        %     % Collect all the state points at each step
        %     obj.stateHistory = [obj.stateHistory, newStates];
        % 
        % end



        % function status = synthesizeLocalControllers(obj,errorDynamicsType,nuBar,rhoBar)
        % 
        %     % Here, we synthesize the local controllers for local error
        %     % dynamics to optimize the passivity properties
        % 
        %     % Error Dynamics Type
        %     % When nu = 0, both methods seems to lead to rho = -1/2 (this will not do)
        %     % When rho = 0, second methods lead to nu = -1/2
        % 
        %     I = eye(3);
        %     O = zeros(3); 
        %     varepsilon = obj.quadrotorParameters(7);
        %     if errorDynamicsType == 1
        %         A = [O,I,O,O;
        %              O,O,obj.quadrotorParameters(6)*(hatmap(e3))^2,O;
        %              O O O I;
        %              O O O O];      % For error dynamics type 1 (not studied yet)
        %     else
        %         A = [O,I,O,O;
        %              O,O,O,O;
        %              O O O 1/varepsilon*I;
        %              O O O O];      % For error dynamics type 2   
        %     end
        % 
        %     B = [O O;
        %          I O;
        %          O O;
        %          O 1/varepsilon*I];
        % 
        %     rhoBarBar = 1/rhoBar;
        % 
        %     nuHat = -0.01;            % nuBar <= nu <= nuHat < 0
        %     rhoHat = 0.01;            % 0 <= rhoHat <= rho <= rhoBar ( or 0 <= rhoBarBar <= rhoBar <= rhoHatBar ) 
        %     rhoHatBar = 1/rhoHat;
        % 
        %     % Set up the LMI problem
        %     solverOptions = sdpsettings('solver','mosek','verbose',0);            
        %     P = sdpvar(12,12,'symmetric'); 
        %     K = sdpvar(6,12,'full'); 
        % 
        %     rhoTilde = sdpvar(1,1,'full'); %Representing: 1/rho
        %     nu = sdpvar(1,1,'full');
        % 
        %     % Basic Constraints
        %     con1 = P >= 0;
        %     %%con2 = trace(P) == 1; % In this setting this is not required actually
        % 
        %     % Approach 4 with rho = prespecified, nu < 0 and nu is free to maximize ???????????????????
        %     I_n = eye(12);
        %     O_n = zeros(12);
        %     DMat = [rhoTilde*I_n];
        %     MMat = [P, O_n];
        %     ThetaMat = [-A*P-P*A'-B*K-K'*B', -I_n+0.5*P; 
        %                 -I_n+0.5*P, -nu*I_n];   % K is the \tilde{L}_{ii}
        %     W = [DMat, MMat; MMat', ThetaMat];
        %     con3 = W >= 0;
        % 
        %     % Some modesty constraints on resulting nu and rho from the local design
        %     con4 = nu >= nuBar;             % -8.1
        %     con5 = rhoTilde >= rhoBarBar;     % 1/4.1
        % 
        %     con6 = nu <= nuHat;
        %     con7 = rhoTilde <= rhoHatBar;
        % 
        %     % To help the global design (\gammaSqBar=10, p_i = 1/N)
        %     gammaSqBar = 10;
        %     con8 = nu >= -gammaSqBar/(1/5) ;     % nu >= -50
        %     con9 = rhoTilde <= 4*gammaSqBar/(1/5);  % rhoBar >= 200
        % 
        %     % Total Cost and Constraints
        %     cons = [con1,con3,con4,con5,con6,con7,con8,con9];
        %     %costFun =  0*(-nu + rhoBar);           % For stabilizing, set coefficient to 0
        %     %costFun = 0.0000001*(-nu + rhoBar);    % Otherwise set to 0.0000001.
        %     costFun = 0.0000001*(- nu + rhoTilde);
        % 
        %     % Solution
        %     sol = optimize(cons,costFun,solverOptions);
        %     status = sol.problem == 0; % sol.info;
        % 
        %     PVal = value(P)
        %     KVal = value(K)
        %     LVal = KVal/PVal
        % 
        %     nuVal = value(nu)
        %     rhoVal = 1/value(rhoTilde)
        % 
        %     disp('TestVal = -nu,rhoTilde/4')
        %     val = [-value(nu),  value(rhoTilde)/4]
        % 
        %     % Updating the information
        %     obj.nu = nuVal;
        %     obj.rho = rhoVal;
        % 
        %     obj.localControllerGains1 = LVal; % Here we need \bar{k}_{i0}^{Local} = 1
        %     obj.localControllerGains2 = LVal; 
        % 
        %     if status == 1
        %         disp(['Synthesis Success at Vehicle ',num2str(obj.quadrotorIndex),'.'])
        %     else
        %         disp(['Synthesis Failed at Vehicle ',num2str(obj.quadrotorIndex),'.'])
        %     end
        % 
        % end

