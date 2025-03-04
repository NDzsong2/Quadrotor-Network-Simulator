classdef Network < handle
    %NETWORK Summary of this class goes here
    %   Detailed explanation goes here

    properties
        networkIndex
        numOfFormations % N
        numOfQuadrotors = [] % [n_1,n_2,...,n_N] 

        formations = [] % all the formations

        % State variables
        time
        error
        cost

        % For plotting
        graphics = [];

    end

    methods

        function obj = Network(indexVal,numOfFormations,numOfQuadrotors,parameters,states,desiredSeparation,noiseMean,noiseStd)

            obj.networkIndex = indexVal;
            obj.numOfFormations = numOfFormations;
            obj.numOfQuadrotors = numOfQuadrotors;

            % Create formations
            formations = [];
            for k = 1:1:numOfFormations
                formation = Formation(k,numOfQuadrotors(k),parameters{k},states{k},desiredSeparation{k},noiseMean{k},noiseStd{k});
                formations = [formations, formation];
            end
            obj.formations = formations;
                        
            obj.time = 0;
            obj.error = zeros(18,1);       
            obj.cost = 0;

        end


        %% Draw the initial network
        function outputArg = drawNetwork(obj,figNum)
            figure(figNum); hold on;
            
            if ~isempty(obj.graphics)
                delete(obj.graphics(1));
                delete(obj.graphics(2)); 
                delete(obj.graphics(3)); 
                delete(obj.graphics(4));
                delete(obj.graphics(5)); 
                delete(obj.graphics(6));
                delete(obj.graphics(7));
            end

            % Draw formations
            for k = 1:1:obj.numOfFormations
                obj.formations(k).drawFormation(figNum);
            end
            
            % Coordinate of the boundary of the bounding box
            lastFormation = obj.formations(obj.numOfFormations);
            posX1 = lastFormation.quadrotors(lastFormation.numOfQuadrotors).states(1) - 2;
            posX2 = obj.formations(1).quadrotors(1).states(1) + 2;

            posY1 = -5; % No lateral changes in Y direction for quadrotor formations
            posY2 = 5;

            posZ1 = -8; % No vertical changes in Z direction for quadrotor formations
            posZ2 = 0;

            
            % Plot the 3 topics, i.e., time, error, cost, on the top left of our simulator
            obj.graphics(1) = text(posX2-9.5,posY1+3,posZ2-13.4,['Time: ',num2str(obj.time)],'FontSize',12);   % -7.5, -2, -13.4
            obj.graphics(2) = text(posX2-9.5,posY1+3,posZ2-12.9,['Error: ',num2str(round(norm(obj.error),1))],'FontSize',12);
            obj.graphics(3) = text(posX2-9.5,posY1+3,posZ2-12.4,['Cost: ',num2str(round(obj.cost,1))],'FontSize',12); 

            % Plot the remaining 4 topics, i.e., position-errors, velocity-errors, angular-errors, angular velocity-errors, on the top left of our simulator
            obj.graphics(4) = text(posX2-7,posY1+3,posZ2-13,['Pos-Error: ',num2str(round(norm(obj.error(1:3)),1))],'FontSize',12);
            obj.graphics(5) = text(posX2-7,posY1+3,posZ2-12.5,['Vel-Error: ',num2str(round(norm(obj.error(4:6)),1))],'FontSize',12);
            obj.graphics(6) = text(posX2-7,posY1+3,posZ2-12,['Ang-Error: ',num2str(round(norm(obj.error(7:9)),1))],'FontSize',12);
            obj.graphics(7) = text(posX2-7,posY1+3,posZ2-11.5,['AngVel-Error: ',num2str(round(norm(obj.error(10:12)),1))],'FontSize',12);
            
            % Set the axis
            xlim([posX1 posX2]);
            ylim([posY1 posY2]);
            zlim([posZ1 posZ2]);

        end


        %% Errors & system statesx update
        function outputArg = update(obj, t, dt, tMax)
            
            % Do the necessary computations/generations to generate the signals to initiate the program
            totalError = zeros(12,1);     

            for k = 1: 1: obj.numOfFormations

                % Error Dynamics Update: Computing Quadrotor Platooning
                % Errors and Controls: 
                % Part 1: position and velocity erros
                obj.formations(k).computeFormationErrors2Part1(t, tMax);
                obj.formations(k).computeControlInputs2Part1(t);    % compute the obj.followerControlInput(1:3) for each quadrotor
                
                % Part 2: orientation and angular velocity errors.
                obj.formations(k).computeFormationErrors2Part2(dt, t);
                obj.formations(k).computeControlInputs2Part2();     % compute the obj.followerControlInput(4:6) for each quadrotor
                
                % Generate the noises (contains the Xi term)
                obj.formations(k).generateNoises();

                % Then, using the new control input by the new errors, we update the states.
                formationError = obj.formations(k).update(t, dt, tMax);
                totalError = totalError + formationError;

            end

            % Update the time, error and cost
            costSoFar = obj.cost^2 * obj.time;
            obj.time = obj.time + dt;

            obj.error = totalError;
            costSoFar = costSoFar + (norm(obj.error))^2 * dt;
            obj.cost = sqrt(costSoFar/obj.time);
            
        end
        

        %% Redraw the network (when quadrotors move)
        function outputArg = redrawNetwork(obj,figNum)
            
            % Redraw formations
            for k = 1:1:obj.numOfFormations
                obj.formations(k).redrawFormation(figNum); % Redraw the formations by calling the redrawPlatoon method in platoon class
            end

            % Update the 3 topics of the figure
            if ~isempty(obj.graphics)
                delete(obj.graphics(1));
                delete(obj.graphics(2)); 
                delete(obj.graphics(3)); 
                delete(obj.graphics(4));
                delete(obj.graphics(5)); 
                delete(obj.graphics(6));
                delete(obj.graphics(7));

                % Coordinate of the boundary of the bounding box
                lastFormation = obj.formations(obj.numOfFormations);
                posX1 = lastFormation.quadrotors(lastFormation.numOfQuadrotors).states(1) - 2;
                posX2 = obj.formations(1).quadrotors(1).states(1) + 2;

                posY1 = -5;
                posY2 = 5;
                
                posZ1 = -8;
                posZ2 = 0;
            
            
                % Plot the 3 topics, i.e., time, error, cost, on the top left of our simulator
                obj.graphics(1) = text(posX2-9.5,posY1+3,posZ2-13.4,['Time: ',num2str(obj.time)],'FontSize',12);
                obj.graphics(2) = text(posX2-9.5,posY1+3,posZ2-12.9,['Error: ',num2str(round(norm(obj.error),1))],'FontSize',12);
                obj.graphics(3) = text(posX2-9.5,posY1+3,posZ2-12.4,['Cost: ',num2str(round(obj.cost,1))],'FontSize',12); 

                % Plot the remaining 4 topics, i.e., position-errors, velocity-errors, angular-errors, angular velocity-errors, on the top left of our simulator
                obj.graphics(4) = text(posX2-7,posY1+3,posZ2-13,['Pos-Error: ',num2str(round(norm(obj.error(1:3)),1))],'FontSize',12);
                obj.graphics(5) = text(posX2-7,posY1+3,posZ2-12.5,['Vel-Error: ',num2str(round(norm(obj.error(4:6)),1))],'FontSize',12);
                obj.graphics(6) = text(posX2-7,posY1+3,posZ2-12,['Ang-Error: ',num2str(round(norm(obj.error(7:9)),1))],'FontSize',12);
                obj.graphics(7) = text(posX2-7,posY1+3,posZ2-11.5,['AngVel-Error: ',num2str(round(norm(obj.error(10:12)),1))],'FontSize',12);
            
                % Set the axis
                xlim([posX1 posX2]);
                ylim([posY1 posY2]);
                zlim([posZ1 posZ2]);

            end

        end

        
        %% Here, we basically have to derive each leader's trajectory
        function outputArg = generateLeadersControlProfiles(obj, tVals, vxVals, vyVals)
            
            % vVals and aVals computation
            vVals = [];
            t_and_uVals = []; 
            for k = 1:1:length(tVals)
                vVals = [vVals; [vxVals(k), vyVals(k), 0]];
                t_and_uVals = [t_and_uVals; tVals(k)', [0 0 0]];  % The acceleration of the leader is [0 0 0]' in the last period of time.
            end 

            % Setting initial velocities
            for k = 1:1:obj.numOfFormations
                obj.formations(k).quadrotors(1).plannedLeaderStates = vVals;
                obj.formations(k).quadrotors(1).states(4:6) = vVals(1,:)';   % initial leader states (velocity)
                obj.formations(k).quadrotors(1).plannedControls = t_and_uVals;
            end
           
        end


        %% Controller computation
        function output = loadFormationControllers(obj,errorDynamicsType,isCentralized,issMS,isOnlyStabilizing,gammaSqBar,nuBar,rhoBar,pVals)
            for k = 1:1:obj.numOfFormations

                % Controller Types:
                % There can be three factors that determines the controller
                % type: 
                % (i) Centralized/Decentralized, 
                % (ii) Stabilizing/Robust
                % (iii) Error Dynamics Type
                
                if errorDynamicsType == 1           % Error dynamics formulation I
                    if isCentralized == 1           % Centralized
                        if isOnlyStabilizing == 1   % Only Stabilizing
                            status = obj.formations(k).centralizedStabilizingControllerSynthesis1(nuBar,rhoBar);
                        else                        % Robust
                            status = obj.formations(k).centralizedRobustControllerSynthesis1(nuBar,rhoBar,gammaSqBar);
                        end
                    else                            % Decentralized
                        if isOnlyStabilizing == 1   % Only Stabilizing
                            status = obj.formations(k).decentralizedStabilizingControllerSynthesis1(nuBar,rhoBar);
                        else                        % Robust
                            status = obj.formations(k).decentralizedRobustControllerSynthesis1(nuBar,rhoBar,gammaSqBar);
                        end
                    end
                else                                % Error dynamics formulation II
                    if isCentralized == 1 && ~issMS % Centralized & Not DSS
                        if isOnlyStabilizing == 1   % Only Stabilizing
                            status = obj.formations(k).centralizedStabilizingControllerSynthesis2(nuBar,rhoBar);
                        else                        % Robust
                            status = obj.formations(k).centralizedRobustControllerSynthesis2(pVals(k,:)); % nuBar,rhoBar,gammaSqBar are no longer needed
                        end
                    elseif ~isCentralized == 1 && ~issMS    % Decentralized & Not DSS
                        if isOnlyStabilizing == 1           % Only Stabilizing
                            status = obj.formations(k).decentralizedStabilizingControllerSynthesis2(nuBar,rhoBar);
                        else                                % Robust
                            status = obj.formations(k).decentralizedRobustControllerSynthesis2(pVals(k,:));
                        end
                    elseif ~isCentralized == 1 && issMS     % Decentralized & DSS
                                                            % Robust
                            status = obj.formations(k).decentralizedRobustControllerSynthesissMS2(pVals(k,:));   % This is the method we are working on.                         
                    end
                end
                
                % Success or Failure
                if status == 1
                    disp(['Synthesis Success at Platoon ',num2str(k),'.']);
                else
                    disp(['Synthesis Failed at Platoon ',num2str(k),'.']);
                end
                
            end
        end
        

        
        function pVals = optimizeCodesignParameters(obj,isCentralized,issMS)
            pVals = [];
            for k = 1:1:obj.numOfFormations
                pVals_k = obj.formations(k).optimizeCodesignParameters(isCentralized,issMS);
                pVals = [pVals; pVals_k];
            end
        end
    
    
    
    end
end