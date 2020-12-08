classdef MapPostProcessing
    % This class is used for map comparison between the estimated map and
    % a groundtruth map
    %
    % Methods:
    %  	MapPreprocessing(estMap)
    %       This is the constructor of the class
    
    % Nils Rottmann (Nils.Rottmann@rob.uni-luebeck.de)
    % 11.03.2019
    
    properties
        % Storage capacities
        EstMap;         % The estimated map
        DP;             % The optimized dominant points
        CutDP;          % Cutted optimized dominant points
        ClosedDP;       % The points of the closed graph
        A;              % Incident matrix
        CutA;           % Incident matrix for cutted DPs
        Cut_indices;    % Indices which are cutted
        Circumference;  % Estimated Circumference
        
        % Parameter Mapping
        L_nh;
        C_max;
        M;
        
        % Flags
        FlagClosedMap;
    end
    
    methods
        function obj = MapPostProcessing()
            % This is the constructor of the class
            
            % Allocate variables
            obj.DP;   
            obj.A;
            
            obj.Circumference = 0;
            obj.Cut_indices = 0;
            
            % Get parameter
            out = get_config('mapping');
            obj.L_nh = out.l_nh;
            obj.C_max = out.c_max;
            obj.M = out.M;
            
            % Set Flags on false
            obj.FlagClosedMap = false;
        end
           
        function [obj] = cutGraph(obj) 
            % This methods throws away the beginning and end of the graph
            % (DPs) which could not be optimized. On both sides these
            % should be lengths of apprx. L_nh.
            %
            % Syntax:
            %       [obj] = CutGraph(obj) 
            %
            
            % Clear beginning and end
            [~,idx11] = min(obj.A(:,length(obj.DP(1,:)):end)); 	% First loop closing constraint
            idx1 = min(idx11);
            [~,idx22] = max(obj.A(:,length(obj.DP(1,:)):end));	% Last loop closing constraint
            idx2 = max(idx22);
            obj.CutDP = obj.DP(:,idx1:idx2);
            A_tmp = obj.A(idx1:idx2,:);
            A_tmp(:,idx2:length(obj.DP(1,:))-1) = [];
            A_tmp(:,1:idx1-1) = [];
            obj.CutA = A_tmp;
            obj.Cut_indices = idx1:idx2;
        end
    
        function [obj] = closeGraph(obj) 
            % This methods closes the cutted DPs
            %
            % Syntax:
            %       [obj] = CutGraph(obj) 
            %
            
            J = length(obj.CutDP(1,:));
            j = 1;
            while ~exist('X_closed') && j <= (J-3)
                X_tmp = obj.CutDP(1:2,j:end) - obj.CutDP(1:2,j);
                S_tmp = zeros(length(X_tmp(1,:)),1);
                for i=1:1:length(X_tmp(1,:))
                    S_tmp(i) = norm(X_tmp(:,i));
                end
                [pks, locs] = findpeaks(-S_tmp);
                % [~,idx] = max(pks);
                % Calculate circumferences for the peak locations
                U_pks = zeros(length(pks),1);
                for i=1:1:length(pks)
                    for ii=1:1:locs(i)-1
                        U_pks(i) = U_pks(i) + norm(obj.CutDP(1:2,j+ii)-obj.CutDP(1:2,j+(ii-1)));
                    end
                end
                [minU,idx] = min(abs(obj.Circumference - U_pks));
                if (abs(pks(idx)) < 0.5)
                    X_closed = [obj.CutDP(:,j:j+locs(idx)-2),obj.CutDP(:,j)];
                    break
                end
                j = j + 1;
            end
            if ~exist('X_closed')
                warning('Could not generate closed loop map!')
                obj.FlagClosedMap = true;
            else
                obj.ClosedDP = X_closed;
            end
        end
        
        function obj = generatePolyMap(obj)
            if obj.FlagClosedMap
                obj.EstMap = [];
            else
                obj.EstMap = genPolyMap(obj.ClosedDP(1,:),obj.ClosedDP(2,:));
            end
        end
        
        function [obj] = generateGridMap(obj)
            % Resolution of the map
            resolution = 20;
            
            % Create the occupancy grid map
            width = max(obj.ClosedDP(1,:)) - min(obj.ClosedDP(1,:)) + 2;
            height =  max(obj.ClosedDP(2,:)) - min(obj.ClosedDP(2,:)) + 2;
            obj.EstMap = binaryOccupancyMap(width,height,resolution);
            
            % Specify grid world location
            obj.EstMap.GridLocationInWorld(1) = min(obj.ClosedDP(1,:)) - 1;
            obj.EstMap.GridLocationInWorld(2) = min(obj.ClosedDP(2,:)) - 1;

            % Fill the grid map
            for i=1:1:obj.EstMap.GridSize(1)
                for j=1:1:obj.EstMap.GridSize(2)
                    xy = grid2world(obj.EstMap, [i j]);
                    isOccupied = inpolygon(xy(1),xy(2),obj.ClosedDP(1,:),obj.ClosedDP(2,:));
                    setOccupancy(obj.EstMap,xy,isOccupied);
                end
            end
        end
        
%         function [obj] = generateGridMap(obj)
%             % Resolution of the map
%             resolution = 20;
%             
%             % Create the occupancy grid map
%             width = max(obj.CutDP(1,:)) - min(obj.CutDP(1,:)) + 2;
%             height =  max(obj.CutDP(2,:)) - min(obj.CutDP(2,:)) + 2;
%             obj.EstMap = binaryOccupancyMap(width,height,resolution);
%             
%             % Specify grid world location
%             obj.EstMap.GridLocationInWorld(1) = min(obj.CutDP(1,:)) - 1;
%             obj.EstMap.GridLocationInWorld(2) = min(obj.CutDP(2,:)) - 1;
%             
%             % Create matrix with p=0.5 values
%             gridMat = ones(size(occupancyMatrix(obj.EstMap))) * 0.5;
%             
%             % Create rotation matrix for rays
%             R_out = [cos(-pi/2) -sin(-pi/2); sin(-pi/2) cos(-pi/2)];
%             R_in = [cos(pi/2) -sin(pi/2); sin(pi/2) cos(pi/2)];
%             
%             N = length(obj.CutDP(1,:));
%             
%             disp(['Start to check ',num2str(N-1),' arcs!'])
%             for i=2:1:N
%                 % Create vector between dominant points
%                 v = obj.CutDP(:,i) - obj.CutDP(:,i-1);
%                 lines = round(norm(v) * 2 * resolution);
%                 
%                 % Get a temporay occupancyMatrix
%                 tmpMat = zeros(size(gridMat));
%                 
%                 for j=0:1:lines
%                     p1 = obj.CutDP(:,i-1) + (j/lines) * v;
%                     
%                     % Outside of the closed environment
%                     counter = 0;
% 
%                     while true
%                         % Get next point from the side ray
%                         p2 = p1 + (counter/(2*resolution)) * (R_out*(v/norm(v)));
%                         if (p2(1) > obj.EstMap.XWorldLimits(1) && p2(1) < obj.EstMap.XWorldLimits(2) ...
%                                 && p2(2) > obj.EstMap.YWorldLimits(1) && p2(2) < obj.EstMap.YWorldLimits(2))
%                             
%                             tmpIdx = world2grid(obj.EstMap,p2');
%                             tmpMat(tmpIdx(1),tmpIdx(2)) = -1;   
%                             
%                         else
%                             break
%                         end
%                         counter = counter + 1;
%                     end
%                     
%                     % Inside the closed environment
%                     counter = 1;
%  
% 
%                     while true
%                         % Get next point from the side ray
%                         p2 = p1 + (counter/(2*resolution)) * (R_in*(v/norm(v)));
%                         if (p2(1) > obj.EstMap.XWorldLimits(1) && p2(1) < obj.EstMap.XWorldLimits(2) ...
%                                 && p2(2) > obj.EstMap.YWorldLimits(1) && p2(2) < obj.EstMap.YWorldLimits(2))
%                             
%                             tmpIdx = world2grid(obj.EstMap,p2');
%                             tmpMat(tmpIdx(1),tmpIdx(2)) = 1;
%                             
%                         else
%                             break
%                         end
%                         counter = counter + 1;
%                     end       
%                 end              
%                 gridMat = gridMat + tmpMat * 0.5 * (1/lines);
%                 
%                 % Display something
%                 disp([num2str(i-1),'/',num2str(N-1)])
%             end
%             
%             for i=1:1:obj.EstMap.GridSize(1)
%                 for j=1:1:obj.EstMap.GridSize(2)
%                     if (gridMat(i,j) > 0.5)
%                         setOccupancy(obj.EstMap,[i j],1,'grid');
%                     else
%                         setOccupancy(obj.EstMap,[i j],0,'grid');
%                     end
%                 end
%             end
%         end
    end  
end