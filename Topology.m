classdef Topology < handle
    %UNTITLED5 Summary of this class goes here
    %   Detailed explanation goes here

    properties
        startNodes
        endNodes
        
        numOfNodes 
        graph  % store the topology as a matlab digraph variable

        inNeighbors
        outNeighbors
        
        % formations Formation
        
        % quadrotors = []
    end

    methods



        function obj = Topology(n_k, startNodes, endNodes, nodeNames)  
            if nargin == 1
%                 [startNodes,endNodes,nodeNames] = obj.generateBidirectionalTopology(n_k);
%                 [startNodes,endNodes,nodeNames] = obj.generateAUniformTopology(n_k);
                 [startNodes,endNodes,nodeNames] = obj.generateInitialTopology(n_k);
%                 [startNodes,endNodes,nodeNames] = obj.generateABlankTopology(n_k);
%                 [startNodes,endNodes,nodeNames] = obj.generateAFullyConnectedTopology(n_k);
            end
            obj.startNodes = startNodes;
            obj.endNodes = endNodes; 
            obj.numOfNodes = n_k;

            weights = ones(length(startNodes), 1);
           
            % Here, we need the digraph for further compute the graph matrices
            obj.graph = digraph(startNodes, endNodes, weights, nodeNames);
            obj.loadNeighborSets(); 

            % obj.formations = Formation;
                        
        end


        function [startNodes,endNodes,nodeNames] = generateBidirectionalTopology(obj, NBar)
            startNodes = [];
            endNodes = [];
            for i = 1:1:NBar
                nodeNames{i} = num2str(i-1);
                % From node i, there are links going to its near neighbors (predecessor & successor)
                for j = 1:1:NBar
                    if j ~= 1 && j ~= i 
                        if abs(i-j) <= 1 % If i and j are adjacent, we add a link from i to j (bidirectional)
                            startNodes = [startNodes, i];
                            endNodes = [endNodes, j];
                        end
                    end
                end

            end
        end


        function [startNodes,endNodes,nodeNames] = generateAUniformTopology(obj, n_k)
            startNodes = [];
            endNodes = [];
            for i = 1:1:n_k
                nodeNames{i} = num2str(i-1);
                if i <= 2
                    startNodes = [startNodes, i];
                    endNodes = [endNodes, i+1];
                elseif i > 2 && i < n_k
                    startNodes = [startNodes, i,i];
                    endNodes = [endNodes, i+1,i-1];
                elseif i == n_k
                    startNodes = [startNodes, i];
                    endNodes = [endNodes, i-1];
                end
            end
        end 


        function [startNodes,endNodes,nodeNames] = generateInitialTopology(obj, NBar)
            startNodes = [];
            endNodes = [];
            for i = 1:1:NBar
                nodeNames{i} = num2str(i-1);
                % state_i = obj.formations.quadrotors(i).states(1:3);
                % From node i, there can be links going to all other NBar nodes 
                for j = 1:1:NBar
                    % Add links from leader to followers
                    if i == 1 && j~=i
                        startNodes = [startNodes, i];
                        endNodes = [endNodes, j];
                    end
                    if j~=1 && j~=i 
                        % state_j = obj.formations.quadrotors(j).states(1:3);
                        % if norm(state_i-state_j) <= 3
                        %     startNodes = [startNodes,i];
                        %     endNodes = [endNodes,j];
                        % end
                        % if abs(i-j)==1 || abs(i-j)==3 % If i and j are adjacent, add a link from i to j
                        %     startNodes = [startNodes,i];
                        %     endNodes = [endNodes,j];
                        % elseif abs(i-j)>5 && abs(i-j)<=7
                        %     startNodes = [startNodes,i];
                        %     endNodes = [endNodes,j];
                        % end
                        % elseif rand(1) <= 0.8*(1/abs(i-j))
                        %     startNodes = [startNodes,i];
                        %     endNodes = [endNodes,j];
                        if abs(i-j) <= 3 % If i and j are adjacent, add a link from i to j
                            startNodes = [startNodes,i];
                            endNodes = [endNodes,j];
                        % elseif abs(i-j)>5 && abs(i-j)<=7
                        %     startNodes = [startNodes,i];
                        %     endNodes = [endNodes,j];
                        end
                    end
                end

            end
        end

        function [startNodes,endNodes,nodeNames] = generateABlankTopology(obj,NBar)
            startNodes = [];
            endNodes = [];
            for i = 1:1:NBar
                nodeNames{i} = num2str(i-1);
                startNodes = [startNodes,1];
                endNodes = [endNodes,i];
            end
        end


        function [startNodes,endNodes,nodeNames] = generateAFullyConnectedTopology(obj,NBar)
            startNodes = [];
            endNodes = [];
            for i = 1:1:NBar
                nodeNames{i} = num2str(i-1);
                % From node i, there can be links going to all other NBar nodes 
                for j = 1:1:NBar
                    if j ~= 1 && j~=i 
                        startNodes = [startNodes,i];
                        endNodes = [endNodes,j];
                    end
                end
            end
        end


        function neighborSets = loadNeighborSets(obj)

            for i = 1:1:obj.numOfNodes
                inNeighbors{i} = predecessors(obj.graph, i);
                outNeighbors{i} = successors(obj.graph, i);
            end
            obj.inNeighbors = inNeighbors;
            obj.outNeighbors = outNeighbors; 
                        
        end

        
       
    end
end







