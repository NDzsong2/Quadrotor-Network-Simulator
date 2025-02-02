classdef Formation < handle
    %FORMATION Summary of this class goes here
    %   Detailed explanation goes here
    
    properties
        % Indices
        formationIndex            % k
        
        % parameters
        numOfQuadrotors           % n
        
        quadrotors = []           % this array holds all the quadrotor class objects created. 
        topology
       
        graphics1 = []
        graphics2 = []

        K                         % controller gains obtained 
    end
    
    methods

        function obj = Formation(k,n_k,parameters,states,desiredSeparation,noiseMean,noiseStd)

            obj.formationIndex = k;
            obj.numOfQuadrotors = n_k;
            
            % Create quadrotors in a formation
            quadrotors = [];
            for i = 1:1:n_k
                % Create an object from the Quadrotor class
                quadrotor = Quadrotor(k,i,parameters(:,i),states(:,i),desiredSeparation(:,i),noiseMean(:,i),noiseStd(:,i));
                quadrotors = [quadrotors, quadrotor];
            end
            obj.quadrotors = quadrotors;

            % obj.topology = Topology(n_k); % Generate a topology
            % obj.updateNeighbors();   % update the neighbor information of each quadrotor object inside obj.quadrotors based on obj.topology
            % obj.loadDefaultControllerGains(); % based on the neighbor connections, load some controller gains

        end
        

        % function outputArg = updateNeighbors(obj)
        %     for i = 1:1:obj.numOfQuadrotors
        %         obj.quadrotors(i).inNeighbors = obj.topology.inNeighbors{i};
        %         obj.quadrotors(i).outNeighbors = obj.topology.outNeighbors{i};
        % 
        %         desiredSeperations = []; % to store d_ij values
        %         for j = 1:1:obj.numOfQuadrotors
        %             d_ij = obj.quadrotors(i).desiredSeparation - obj.quadrotors(j).desiredSeparation;
        %             desiredSeperations = [desiredSeperations, d_ij];
        %         end
        %         obj.quadrotors(i).desiredSeparations = desiredSeperations;
        %     end
        % end


        function outputArg = drawFormation(obj,figNum)
            figure(figNum); hold on; 

            % Draw quadrotors
            for i = 1:1:obj.numOfQuadrotors
                obj.quadrotors(i).drawQuadrotor(figNum);
            end
            
            % use the obj.function() to add the topology (Because this is the layer that we can get access to the quadrotor class)
            % obj.drawTopology(figNum) 
        end


        % Draw the topology for a fixed graph
        function outputArg = drawTopology(obj,figNum)
            figure(figNum); hold on;

            if ~isempty(obj.graphics1)
                delete(obj.graphics1);
                delete(obj.graphics2);
            end

            numOfLinks = length(obj.topology.startNodes);
            for i = 1:1:numOfLinks
                % Draw a link
                startVehicleIndex = obj.topology.startNodes(i);
                endVehicleIndex = obj.topology.endNodes(i);

                startPos = obj.vehicles(startVehicleIndex).states(1) - obj.vehicles(startVehicleIndex).vehicleParameters(2)/2;
                endPos = obj.vehicles(endVehicleIndex).states(1) - obj.vehicles(endVehicleIndex).vehicleParameters(2)/2;
                midPos = (startPos + endPos)/2;
                midPointHeight = -3*sign(startPos-endPos)+0.05*abs(startPos-endPos)+1.5*(startPos<endPos); % 4

                startPosY = obj.vehicles(startVehicleIndex).vehicleParameters(2)*3/8;
                endPosY = obj.vehicles(endVehicleIndex).vehicleParameters(2)*3/8;
                %obj.graphics(i) = plot([startPos,midPos,endPos],[startPosY,midPointHeight,endPosY],'-b');

                % Plotting the Spline
                x = [startPos,midPos,endPos];
                y = [startPosY,midPointHeight,endPosY];
                stepSize = (endPos-startPos)/20; 
                xx = startPos:stepSize:endPos;
                yy = spline(x,y,xx);
                obj.graphics1(i) = plot(xx,yy,'-b');
            
                % Plotting the arrowHead (polyshape)
                polyPosX = midPos;
                polyPosY = midPointHeight;
                polySize = 0.3;
                polyVertX = [-0.5,1,-0.5];
                polyVertY = [0.5,0,-0.5];
                if polyPosY < 0
                    polyVertX = -polyVertX;
                end
                arrowHead = polyshape(polyPosX+polySize*polyVertX,polyPosY+polySize*polyVertY);
                obj.graphics2(i) = plot(arrowHead,'EdgeColor','k','FaceColor','b');
            end            
        end


        function outputArg = redrawFormation(obj,figNum)
            figure(figNum); hold on; 
                        
            % Redraw quadrotors
            for i = 1:1:obj.numOfQuadrotors
                obj.quadrotors(i).redrawQuadrotor(figNum);
            end
            
            % Redraw the topology by calling redrawTopology method, based on the newly updated states of the vehicles
            % obj.redrawTopology(figNum) 
        end


        % Redraw the topology for a fixed graph
        function outputArg = redrawTopology(obj,figNum)
            figure(figNum); hold on;
                 
            numOfLinks = length(obj.topology.startNodes);
            
            for i = 1:1:numOfLinks
                if ~isempty(obj.graphics1(i))
                    delete(obj.graphics1(i));
                    delete(obj.graphics2(i));

                    % Redraw a link
                    startVehicleIndex = obj.topology.startNodes(i);
                    endVehicleIndex = obj.topology.endNodes(i);
                    
                    startPos = obj.vehicles(startVehicleIndex).states(1) - obj.vehicles(startVehicleIndex).vehicleParameters(2)/2;
                    endPos = obj.vehicles(endVehicleIndex).states(1) - obj.vehicles(endVehicleIndex).vehicleParameters(2)/2;
                    midPos = (startPos + endPos)/2;
                    midPointHeight = -3*sign(startPos-endPos)+0.05*abs(startPos-endPos)+1.5*(startPos<endPos); % 4
                    
                    startPosY = obj.vehicles(startVehicleIndex).vehicleParameters(2)*3/8;
                    endPosY = obj.vehicles(endVehicleIndex).vehicleParameters(2)*3/8;
                    

                    % Plotting the Spline
                    x = [startPos,midPos,endPos];
                    y = [startPosY,midPointHeight,endPosY];
                    stepSize = (endPos-startPos)/20; 
                    xx = startPos:stepSize:endPos;
                    yy = spline(x,y,xx);
                    obj.graphics1(i) = plot(xx,yy,'-b');
                
                    % Plotting the arrowHead (polyshape)
                    polyPosX = midPos;
                    polyPosY = midPointHeight;
                    polySize = 0.3;
                    polyVertX = [-0.5,1,-0.5];
                    polyVertY = [0.5,0,-0.5];
                    if polyPosY < 0
                        polyVertX = -polyVertX;
                    end
                    arrowHead = polyshape(polyPosX+polySize*polyVertX,polyPosY+polySize*polyVertY);
                    obj.graphics2(i) = plot(arrowHead,'EdgeColor','k','FaceColor','b');
                end
            end            
        end


        function outputArg = generateNoises(obj)
            for i = 1:1:obj.numOfQuadrotors
                obj.quadrotors(i).generateNoise();
            end
        end


        
    end
end

