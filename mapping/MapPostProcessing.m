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
        ClosedDP;       % Closed dominant points
        A;              % Incident matrix
        CutA;           % Incident matrix for cutted DPs
        Cut_indices;    % Indices which are cutted
        Circumference;  % Estimated Circumference
        
        % Parameter
        % Mapping
        L_nh;
        C_min;
        M;
    end
    
    methods
        function obj = MapPostProcessing(DP,A)
            % This is the constructor of the class
            
            % Allocate variables
            obj.DP = DP;   
            obj.A = A;
            
            obj.Circumference = 0;
            obj.Cut_indices = 0;
            
            % Get parameter
            out = get_config('mapping');
            obj.L_nh = out.l_nh;
            obj.C_min = out.c_min;
            obj.M = out.M;
        end
        
        function results = compareMaps(obj,trueMap,mode)
            % This is the main function of the class which runs the map
            % comparison functions
            %
            % Syntax:
            %       results = CompareMaps(estMap,trueMap,mode)
            %
            % Input:
            %   estMap:    	poly map of the estimated map
            %   trueMap:    poly map of the true map
            %   mode:       comparison mode
            %       1:      
            %
            % Output:
            %   results:    structure with results
            %       Xclosed:   
            %
            
            disp('Compare estimated map with groundtruth. This can take somewhile ...')
            optAcc = 10^(-6);
            
            % Transform maps in set of points
            X1 = [obj.EstMap.x; obj.EstMap.y];
            X2 = [trueMap.x; trueMap.y];
            
            if (mode == 1) || (mode == 3) || (mode == 4)
                % Generate Matrices for comparison
                X{1} = [X1(:,1:end-1) X1];
                X{2} = [X2(:,1:end-1) X2];

                % Find similar points between the maps
                param.l_nh = obj.L_nh;
                param.c_min = obj.C_min;
                param.m = obj.M;
                [SP] = findSPs(X,param);

                % Pick the points and generate subsets
                N = length(SP(:,1));
                M = length(SP(1,:));
                X1_Subset = [];
                X2_Subset = [];
                for i=1:1:N
                    for j=1:1:M
                        if SP(i,j) == 1
                            X1_Subset = [X1_Subset, X{1}(:,i)];
                            X2_Subset = [X2_Subset, X{2}(:,j)];
                        end
                    end
                end

                % Use Horn Schema
                if ~isempty(X1_Subset) && ~isempty(X2_Subset)
                    [regParams,~,~]=absor(X1_Subset,X2_Subset);
                    X1 = regParams.R*X1 + regParams.t;
                end

                % Compare the maps
                Poly1 = polyshape(X1(1,:),X1(2,:),'Simplify',false);
                Poly2 = polyshape(X2(1,:),X2(2,:),'Simplify',false);
                Comp = area(xor(Poly1,Poly2))/area(union(Poly1,Poly2));
            end
            if (mode == 2) || (mode == 3) || (mode == 4)
                if mode == 2
                    X1 = X1 + (X2(:,1) - X1(:,1));
                end
                Poly1 = polyshape(X1(1,:),X1(2,:),'Simplify',false);
                Poly2 = polyshape(X2(1,:),X2(2,:),'Simplify',false);
                % Start by adjusting both maps onto each other manually
                [x_center_1,y_center_1] = centroid(Poly1);
                [x_center_2,y_center_2] = centroid(Poly2);
                dv = -[x_center_1 - x_center_2, y_center_1 - y_center_2];
                Poly1 = translate(Poly1,dv(1),dv(2));
                refpoint = [x_center_2, y_center_2];
                Comp = area(xor(Poly1,Poly2))/area(union(Poly1,Poly2));
 %               [Poly1,Comp] = MapPostProcessing.gradientDescentComplete(Poly1,Poly2,Comp,refpoint,10^(-3));
                if mode == 4
                    % Get some different rotations
                    phi = 0:1:360;
                    Poly1_tmp = cell(length(phi),1);
                    Comp_tmp = zeros(length(phi),1);
                    for ii=1:1:length(phi)
                        Poly1_tmp{ii} = rotate(Poly1,phi(ii));
                        Comp_tmp(ii) = area(xor(Poly1_tmp{ii},Poly2))/area(union(Poly1_tmp{ii},Poly2));
                        [Poly1_tmp{ii},Comp_tmp(ii)] = MapPostProcessing.gradientDescentComplete(Poly1_tmp{ii},Poly2,Comp_tmp(ii),refpoint,0.001);
                    end
                    [~,idx] = min(Comp_tmp);
                    [Poly1,Comp] = MapPostProcessing.gradientDescentComplete(Poly1_tmp{idx},Poly2,Comp_tmp(idx),refpoint,optAcc);
                end
            end
            % Try genetic algorithms
            if (mode == 5)
                % Generate Polygons
                Poly1 = polyshape(X1(1,:),X1(2,:),'Simplify',false);
                Poly2 = polyshape(X2(1,:),X2(2,:),'Simplify',false);
                % Start by adjusting both maps onto each other manually
%                 [x_center_1,y_center_1] = centroid(Poly1);
                [x_center_2,y_center_2] = centroid(Poly2);
%                 dv = -[x_center_1 - x_center_2, y_center_1 - y_center_2];
%                 Poly1 = translate(Poly1,dv(1),dv(2));
                refpoint = [x_center_2, y_center_2];
                % Use genetic algorithms to find a good first guess
                trans = ga(@compCostMaps,3);
                % Transform map to best guess
                Poly1 = rotate(Poly1,trans(3),refpoint);
                Poly1 = translate(Poly1,trans(1),trans(2));
                compCost = area(xor(Poly1,Poly2))/area(union(Poly1,Poly2));
                % Optimize with gradient descent
                [Poly1,Comp] = MapPostProcessing.gradientDescentComplete(Poly1,Poly2,compCost,refpoint,optAcc);
            end
            if (mode == 6)
                % Generate Polygons
                Poly1 = polyshape(X1(1,:),X1(2,:),'Simplify',false);
                Poly2 = polyshape(X2(1,:),X2(2,:),'Simplify',false);
                % Start by adjusting both maps onto each other manually
                [x_center_1,y_center_1] = centroid(Poly1);
                [x_center_2,y_center_2] = centroid(Poly2);
                dv = -[x_center_1 - x_center_2, y_center_1 - y_center_2];
                Poly1 = translate(Poly1,dv(1),dv(2));
                refpoint = [x_center_2, y_center_2];
                % Go through phis
                phi = 0:0.1:360;
                Poly1_tmp = cell(length(phi),1);
                Comp_tmp = zeros(length(phi),1);
                for ii=1:1:length(phi)
                    Poly1_tmp{ii} = rotate(Poly1,phi(ii),refpoint);
                    Comp_tmp(ii) = area(xor(Poly1_tmp{ii},Poly2))/area(union(Poly1_tmp{ii},Poly2));
                end
                [~,idx] = min(Comp_tmp);
                [Poly1,Comp] = MapPostProcessing.gradientDescentComplete(Poly1_tmp{idx},Poly2,Comp_tmp(idx),refpoint,optAcc);
            end
            
            
            X1 = [Poly1.Vertices' Poly1.Vertices(1,:)'];

            % Plot results
            h =  findobj('type','figure');
            n = length(h);
            figure(n+1)
            plot(X1(1,:),X1(2,:))
            hold on
            plot(X2(1,:),X2(2,:))
            legend('Map Estimate','Original Map')

            % Put into results
            results.DP = obj.DP;
            results.cutDP = obj.CutDP;
            results.closedDP = obj.ClosedDP;
            results.compDP = X1;
            results.trueDP = X2;
            results.turnedEstPolyMap = genPolyMap(X1(1,:),X1(2,:));
            results.error = Comp;
            
            disp('Comparison completed successfully!')
            
            % Cost function for comparison
            function compCost = compCostMaps(dx)
                Poly_tmp = Poly1;
                Poly_tmp = rotate(Poly_tmp,dx(3),refpoint);
                Poly_tmp = translate(Poly_tmp,dx(1),dx(2));
                compCost = area(xor(Poly_tmp,Poly2))/area(union(Poly_tmp,Poly2));
            end
            
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
                                       
%                 for i=1:1:length(pks)   % The first would be the starting position, thus neglect
%                     if abs(pks(i)) < 0.5
%                         X_closed = [obj.CutDP(:,1:locs(i)-1),obj.CutDP(:,1)];
%                         break
%                     end
%                 end
                j = j + 1;
            end
            if ~exist('X_closed')
                warning('Could not generate closed loop map!')
            else
                obj.ClosedDP = X_closed;
            end
        end
        
        function obj = generatePolyMap(obj)
            obj.EstMap = genPolyMap(obj.ClosedDP(1,:),obj.ClosedDP(2,:));
        end
    end
    
    methods (Static)
        function [Poly1,Comp] = gradientDescentComplete(Poly1,Poly2,Comp,refpoint,acc)
            % Holders
            J_old = zeros(3,1);
            % Params for numerical evaluation
            diff = 1;
            dx = 10^-(6);
            while diff > acc          
                % Generate numerical differetiate
                J = zeros(3,1);
                Poly1_x = translate(Poly1,dx,0);
                Poly1_y = translate(Poly1,0,dx);
                Poly1_phi = rotate(Poly1,dx,refpoint);
                J(1) = (area(xor(Poly1_x,Poly2))/area(union(Poly1_x,Poly2)) - Comp) / dx;
                J(2) = (area(xor(Poly1_y,Poly2))/area(union(Poly1_y,Poly2)) - Comp) / dx;
                J(3) = (area(xor(Poly1_phi,Poly2))/area(union(Poly1_phi,Poly2)) - Comp) / dx;
                Delta_F = J*Comp;
                alpha = (Delta_F)'*(J - J_old)/(norm(J - J_old)^2);
                trans = - alpha * Delta_F;
                for jj=1:1:3
                    if ~isfinite(trans(jj)) || ~isreal(trans(jj)) || isnan(trans(jj))
                        try 
                            trans(jj) = -10^(-6) * Delta_F;
                        catch
                            break
                        end
                    end
                end
                Poly1 = rotate(Poly1,trans(3),refpoint);
                Poly1 = translate(Poly1,trans(1),trans(2));
                Comp_tmp = area(xor(Poly1,Poly2))/area(union(Poly1,Poly2));
                diff = Comp - Comp_tmp;
                Comp = Comp_tmp;
                disp(Comp)
            end
        end
        function [Poly1,Comp] = gradientDescentRotation(Poly1,Poly2,Comp,refpoint,acc)
            % Holders
            J_old = 0;
            % Params for numerical evaluation
            diff = 1;
            dx = 10^-(6);
            while diff > acc         
                % Generate numerical differetiate
                Poly1_phi = rotate(Poly1,dx,refpoint);
                J = (area(xor(Poly1_phi,Poly2))/area(union(Poly1_phi,Poly2)) - Comp) / dx;
                Delta_F = J*Comp;
                alpha = (Delta_F)'*(J - J_old)/(norm(J - J_old)^2);
                trans = - alpha * Delta_F;
                if ~isfinite(trans) || ~isreal(trans) || isnan(trans)
                    trans = 0;
                end
                Poly1 = rotate(Poly1,trans,refpoint);
                Comp_tmp = area(xor(Poly1,Poly2))/area(union(Poly1,Poly2))
                diff = Comp - Comp_tmp;
                Comp = Comp_tmp;
            end
        end
    end
    
end