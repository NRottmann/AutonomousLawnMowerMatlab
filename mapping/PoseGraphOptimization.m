classdef PoseGraphOptimization
    % This class is used for pose graph optimization as presented in (1).
    % Please refer to the manual.
    %
    % Methods:
    %  	PoseGraphOptimization(pathData)
    %       This is tne constructot of the class
    
    % Nils Rottmann (Nils.Rottmann@rob.uni-luebeck.de)
    % 29.11.2018
    
    properties
        PathData;       % Data retieved by driving with the vehicle along the boundary line
        
        L_min;          % Parameter for data pruning
        E_max;
        L_nh;           % Parameter for correlation calculation
        C_min;
        M;

    end
    
    methods
        function obj = PoseGraphOptimization(pathData)
            % This is the constructor of the class
            
            % TODO: Add size check, matrxi has to be 2 x N
            
            % Alloctae 
            obj.PathData = pathData;
            
            % Get Parameters
            out = get_config('mapping');
            obj.L_min = out.l_min;
            obj.E_max = out.e_max;
            obj.L_nh = out.l_nh;
            obj.C_min = out.c_min;
            obj.M = out.M;
        end
        
        function [poseGraph,updatedGraph] = generateMap(obj)
            % This is the main function of the class which runs the mapping
            % algorithm and gives back an map estimate
            %
            % Syntax:
            %       [] = generateMap(obj)
            %
            % Input:
            %   obj:            Object of the this class
            %
            % Output:
            %
            
            % (1) Data Pruning of the odometry data
            disp(['Prune ',num2str(length(obj.PathData(1,:))),' data points ...'])
            pruningParam.e_max = obj.E_max;
            pruningParam.l_min = obj.L_min;
            DP = PoseGraphOptimization.generateDPs(obj.PathData,pruningParam);
            disp(['Data points number reduced to ',num2str(length(DP(1,:))),'!'])
            
            % (2) Generate measurements from the pruned data set
            disp(['Generate measurement data from ',num2str(length(DP(1,:))),' dominant points ...'])
            xi = PoseGraphOptimization.generateMeasurements(DP);
            disp(['Generated measurement data!'])
            
            % (3) Find pairs of DPs for Loop Closure
            disp(['Search for SPs ...'])
            correlationParam.l_nh = obj.L_nh;
            correlationParam.c_min = obj.C_min;
            correlationParam.m = obj.M;
            [SP] = PoseGraphOptimization.generateSPs(DP,correlationParam);
            disp(['Found ',num2str((sum(sum(SP))-length(SP(1,:)))),' similar pairs!'])
            
            % Matlab solution
            % (4) Generate pose graph
            disp(['Generate pose graph ...'])
            poseGraph = robotics.PoseGraph;
            for i=1:1:length(xi(1,:))
                poseGraph.addRelativePose(xi(:,i)');
            end
            disp(['Pose graph generated!'])
            % (5) Add loop closures
            disp(['Add loop closures ...'])
            for i=1:1:length(DP(1,:))
                for j=(1+i):1:length(DP(1,:))
                    if SP(i,j) == 1
                        % TODO: Which kind of information matrix is
                        % required?
                        poseGraph.addRelativePose([0 0 0], [1 0 0 1 0 1], ...
                                                i,j)
                    end
                end
            end
            disp(['Loop closures added!'])
            % (6) Optimize pose graph
            disp(['Optimize pose graph ...'])
            updatedGraph = optimizePoseGraph(poseGraph);
            disp(['Pose graph optimized!'])
        end       
    end
    
    methods (Static)
        function DP = generateDPs(data,param)
            % Function which prunes the given odometry data set
            %
            % Syntax:
            %       DP = generateDPs(data,param)
            %
            % input:
            %   data:   Odometry data (Matrix with 2 x N)
            %   param:  Parameters required by the algorithm
            %           l_min: minimum distance between two DPs
            %           e_max: maximum allowed line error 
            % output:
            %   DP:     Pruned odometry data points (Matrix with 2x M), we
            %           call them DPs (dominant points)
            
            % TODO: Size check
            
            N = length(data(1,:));
            
            DP = data(:,1);
            S = data(:,1);
            for i=2:1:N
                d = norm(DP(:,end) - data(:,i));
                if (d < param.l_min)
                    S = [S data(:,i)];
                else
                    S_tmp = [S, data(:,i)];
                    e = errorLineFit(S);
                    if (e < param.e_max)
                        S = S_tmp;
                    else
                        DP = [DP, S(:,end)];
                        S = [S(:,end),data(:,i)];
                    end
                end
            end  
        end
        
        function xi = generateMeasurements(data)
            % Function to generate the measurement data from a pruned data
            % set of positions. Therefore we define the orientation of 
            % node_i as the orientation of the vector from node_i to node
            % node_i+1
            %
            % input:
            %   data:   data set with pruned data points (e.g. DPs) as a 2
            %   x M matrix
            % output:
            %   xi:     relative poses
            
            % TODO: Size check
            
            M = length(data(1,:));
            
            % TODO: Check if this method is valid
            theta = zeros(1,M-1);
            for i=2:1:M
                v = data(:,i) - data(:,i-1);
                theta(i-1) = atan2(v(2),v(1));
            end
            xi = zeros(3,M-2);
            for i=2:1:(M-1)
                R = [cos(theta(i-1)), -sin(theta(i-1)); ...
                                sin(theta(i-1)), cos(theta(i-1))];
                xi(1:2,i-1) = R' * (data(:,i) - data(:,i-1));
                % Regularization, TODO: Check if this is valid
                xi(3,i-1) = theta(i) - theta(i-1);
                if xi(3,i-1) > pi
                    xi(3,i-1) = xi(3,i-1) - 2*pi;
                elseif xi(3,i-1) < -pi
                    xi(3,i-1) = xi(3,i-1) + 2*pi;
                end              
            end
            
        end
        
        function [SP] = generateSPs(data,param)
            % Calculate the similar points in regard to the given data set
            % and the given parameters
            % 
            % input:
            %   data:           Data set with positions as matrix 2 x N, 
            %                   e.g. DPs
            %   param:          Parameter required for the algorithm
            %       m:  ...
            %       l_nh: ...
            %       c_min: ...
            % output:
            %   SP:             Incident matrix with similar points
            %
            
            % Get length and orientations
            M = length(data(1,:));               % Number of data points
            phi = zeros(M,1);
            l = zeros(M,1);
            for i=3:1:M                          % Go through all DPs
                v = data(:,i) - data(:,i-1);
                phi_tmp = atan2(v(2),v(1));
                phi(i-1) = phi_tmp;                         % Orientation of line segments
                l(i-1) = norm(data(:,i) - data(:,i-1));   	% Length of line segments
            end
            
            % Accumulate Orientations
            phi_cumulated = zeros(M,1);
            l_cumulated = zeros(M,1);
            for i = 2:1:M
              delta_phi = phi(i) - phi(i-1);
                % Regularization
                if abs(delta_phi) > pi
                  if phi(i-1) > 0.0
                    delta_phi = delta_phi + 2*pi;
                  else
                    delta_phi = delta_phi - 2*pi;
                  end
                end
              phi_cumulated(i) = phi_cumulated(i-1) + delta_phi;  
              l_cumulated(i) = l_cumulated(i-1) + l(i);
            end
            
            % Calculate correlation
            % TODO: Here we can improve the findings of the correlation
            % optima
            corr = zeros(M);
            l_evaluation = linspace(-param.l_nh,param.l_nh,param.m);
            for i=1:1:M
                l_cumulated_i = l_cumulated - l_cumulated(i);
                phi_cumulated_i = phi_cumulated - phi_cumulated(i);
                for j=1:1:M
                    l_cumulated_j = l_cumulated - l_cumulated(j);
                    phi_cumulated_j = phi_cumulated - phi_cumulated(j);
                    for k=1:1:param.m
                        corr(i,j) = corr(i,j) ...
                            + (getOrientation(phi_cumulated_i,l_cumulated_i,l_evaluation(k)) ...
                                    - getOrientation(phi_cumulated_j,l_cumulated_j,l_evaluation(k)))^2;
                    end
                    corr(i,j) = corr(i,j) / param.m;
                end
            end
            
            % Calculate similar points
            l_min = param.l_nh;                      % Minimum Length
            l_max = l_cumulated(end) - param.l_nh;  % Maximum Length
            SP = zeros(M);
            for i=1:1:M
                if l_cumulated(i) > l_min && l_cumulated(i) < l_max
                    [pks, locs] = findpeaks(-corr(i,:));
                    locs(abs(pks) > param.c_min) = [];
                    SP(i,locs) = 1;
                end
            end
        end 
    end
end

