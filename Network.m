classdef Network < handle
    %NETWORK Summary of this class goes here
    %   Detailed explanation goes here

    properties
        networkIndex
        numOfFormations % N
        numOfQuadrotors = [] % [n_1,n_2,...,n_N] 

        formations = [] % all the platoons

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
            obj.error = [0;0;0;0;0;0;0;0;0;0;0;0];
            obj.cost = 0;

        end

           
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
            % posY1 = -5;
            % posY2 = 20;
            lastFormation = obj.formations(obj.numOfFormations);
            % posX1 = lastFormation.quadrotors(lastFormation.numOfQuadrotors).states(1)-10;
            % posX2 = obj.formations(1).quadrotors(1).states(1)+10;
            
            xlim([-10 2]);
            ylim([-5 5]);
            zlim([-8 0]);

            % Plot the 3 topics, i.e., time, error, cost, on the top left of our simulator
            % obj.graphics(1) = text(posX1+5,posY2-5,['Time: ',num2str(obj.time)],'FontSize',12);
            % obj.graphics(2) = text(posX1+30,posY2-5,['Error: ',num2str(round(norm(obj.error),1))],'FontSize',12);
            % obj.graphics(3) = text(posX1+55,posY2-5,['Cost: ',num2str(round(obj.cost,1))],'FontSize',12); 
            % 
            % obj.graphics(4) = text(posX1+5,posY2-8,['P-Error: ',num2str(round(obj.error(1),1))],'FontSize',12);
            % obj.graphics(5) = text(posX1+30,posY2-8,['V-Error: ',num2str(round(obj.error(2),1))],'FontSize',12);
            % obj.graphics(6) = text(posX1+55,posY2-8,['A-Error: ',num2str(round(obj.error(3),1))],'FontSize',12);

            obj.graphics(1) = text(-7.5,-2,-13.4,['Time: ',num2str(obj.time)],'FontSize',12);
            obj.graphics(2) = text(-7.5,-2,-12.9,['Error: ',num2str(round(norm(obj.error),1))],'FontSize',12);
            obj.graphics(3) = text(-7.5,-2,-12.4,['Cost: ',num2str(round(obj.cost,1))],'FontSize',12); 

            
            obj.graphics(4) = text(-5,-2,-13,['Pos-Error: ',num2str(round(norm(obj.error(1:3)),1))],'FontSize',12);
            obj.graphics(5) = text(-5,-2,-12.5,['Vel-Error: ',num2str(round(norm(obj.error(4:6)),1))],'FontSize',12);
            obj.graphics(6) = text(-5,-2,-12,['Ang-Error: ',num2str(round(norm(obj.error(7:9)),1))],'FontSize',12);
            obj.graphics(7) = text(-5,-2,-11.5,['AngVel-Error: ',num2str(round(norm(obj.error(10:12)),1))],'FontSize',12);

            % axis([posX1,posX2,posY1,posY2])
        end

        %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
        % 先不考虑这个，先画出上面的network method
        function outputArg = update(obj,t,dt)
            
            % Do the necessary computations/generations to generate the signals to initiate the program
            totalError = zeros(12,1);

            for k = 1:1:obj.numOfFormations

                % Generate the noises
                obj.formations(k).generateNoises();

                % Error Dynamics - I : Computing Platooning Errors and Controls
%                 obj.platoons(k).computePlatooningErrors1();
%                 obj.platoons(k).computeControlInputs1(t);

                % Error Dynamics - II : Computing Platooning Errors and Controls
                % obj.formations(k).computePlatooningErrors2();
                % obj.formations(k).computeControlInputs2(t); 

                % Update the states
                formationError = obj.formations(k).update(t,dt);
                totalError = totalError + formationError;

            end


            % Update the time, error and cost
            costSoFar = obj.cost^2*obj.time;
            obj.time = obj.time + dt;

            
            obj.error = totalError;
            costSoFar = costSoFar + (norm(obj.error))^2*dt;
            obj.cost = sqrt(costSoFar/obj.time);
            
        end
        %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%


        %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
        % 先不考虑这个，先画出上面的network method
        function outputArg = redrawNetwork(obj,figNum)
            
            % Redraw platoons
            for k = 1:1:obj.numOfFormations
                obj.formations(k).redrawFormation(figNum); % Redraw the platoons by calling the redrawPlatoon method in platoon class
            end

            % Update the 3 topics of the figure
            if ~isempty(obj.graphics)
                delete(obj.graphics(1));
                delete(obj.graphics(2)); 
                delete(obj.graphics(3)); 
                delete(obj.graphics(4));
                delete(obj.graphics(5)); 
                delete(obj.graphics(6));

                % Coordinate of the boundary of the bounding box
                posY1 = -5;
                posY2 = 20;
                lastFormation = obj.formations(obj.numOfFormations);
                posX1 = lastFormation.quadrotors(lastFormation.numOfQuadrotors).states(1)-10;
                posX2 = obj.formations(1).quadrotors(1).states(1)+10;
                
                obj.graphics(1) = text(posX1+5,posY2-5,['Time: ',num2str(obj.time)],'FontSize',12);
                obj.graphics(2) = text(posX1+30,posY2-5,['Error: ',num2str(round(norm(obj.error),1))],'FontSize',12);
                obj.graphics(3) = text(posX1+55,posY2-5,['Cost: ',num2str(round(obj.cost,1))],'FontSize',12);           
                
                obj.graphics(4) = text(posX1+5,posY2-8,['P-Error: ',num2str(round(obj.error(1),1))],'FontSize',12);
                obj.graphics(5) = text(posX1+30,posY2-8,['V-Error: ',num2str(round(obj.error(2),1))],'FontSize',12);
                obj.graphics(6) = text(posX1+55,posY2-8,['A-Error: ',num2str(round(obj.error(3),1))],'FontSize',12);  

                axis([posX1,posX2,posY1,posY2])
                % axis([min(posX1,posX2),max(posX1,posX2),posY1,posY2])
            end

        end
        %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%


    end
end