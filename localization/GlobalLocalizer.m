classdef GlobalLocalizer
    % Global Localizer for the lawn mower
    %
    % Methods:
    %   GlobalLocalizer
    %
    % Nils Rottmann (Nils.Rottmann@rob.uni-luebeck.de)
    % 05.03.2019
    
    properties
        % Variables
        DP;
        S;
        
        % Parameters
        sensorPosition;      	% Sensor position
        L_min;                  % minimum length of line segment
        L_nh;                   % Neighbourhood length
        E_max;                  % maximum error for line segment
        U_min;                  % percentage of minimum circumeference before taking a decision
        C_min;                  % Minimum allowed correlation error
        C_diff;                 % Minimum required difference in correlation error
        
        % Map
        PolyMap;
        
    end
    
    methods
        function obj = GlobalLocalizer(polyMap)
            % This is the constructor of the class
            % Syntax:
            %       obj = GlobalLocalizer()
            % Input:
            %   
            % Output:
            % 
            
            %% Allocate the map
            obj.PolyMap = polyMap;
            
            %% Initialize variables
            obj.DP = zeros(2,1);
            obj.S = zeros(2,1);
            
            %% Get parameters
            % Sensor position
            out = get_config('Sensor');
            obj.sensorPosition =  out.posRight;
            % Algorithm
            out = get_config('globalLocalization');
            obj.L_min = out.l_min;
            obj.L_nh = out.l_nh;
            obj.E_max = out.e_max;
            obj.U_min = out.u_min;
            obj.C_min = out.c_min;
            obj.C_diff = out.c_diff;
        end
        
        function [obj,results] = localization(obj,estPose)
            % This is the localization function of the class
            % Syntax:
            %       obj = localization()
            % Input:
            %       estPose:   	Pose estimation for the robot
            % Output:
            %   
            
            % Calculate the estimate of the sensor position from the robots
            % pose
            R = [cos(estPose(3)) -sin(estPose(3)); sin(estPose(3)) cos(estPose(3))];
            x_new = estPose(1:2) + R*obj.sensorPosition;      
            % Calculate distance
            L = norm(x_new - obj.DP(:,end));
            if (L < obj.L_min)          % If distance to small, just add point to line segment
                obj.S = [obj.S, x_new];
                results.foundPosition = false;
            else                        % Else, check if error is still small enough
                S_tmp = [obj.S, x_new];
                e = errorLineFit(S_tmp);
                if (e < obj.E_max)      % Error is small enough
                    obj.S = S_tmp;
                    results.foundPosition = false;
                else                    % New DP, check if we already found a pose estimate
                    obj.DP = [obj.DP, obj.S(:,end)];
                    obj.S = [obj.S(:,end), x_new];
                    % Check the length and compare with circumeference of the map
                    l_DP = 0;
                    for i=2:1:length(obj.DP(1,:))
                        l_DP = l_DP + norm(obj.DP(:,i) - obj.DP(:,i-1));
                    end
                    if l_DP > obj.U_min*(2*obj.L_nh)
                        % Delete last DP if too long
                        if l_DP > (2*obj.L_nh)         
                            obj.DP(:,1) = [];
                        end
                        compParam.c_min = obj.C_min;
                        compParam.c_diff = obj.C_diff;
                        results = GlobalLocalizer.compare(obj.DP,obj.PolyMap,compParam);
                        if results.foundPosition
                            % Calculate robot pose
                            R = [cos(results.estPose(3)) -sin(results.estPose(3)); ...
                                    sin(results.estPose(3)) cos(results.estPose(3))];
                            results.estPose(1:2) = results.estPose(1:2) - R*obj.sensorPosition;
                        end
                    else
                        results.foundPosition = false;
                    end
                end
            end
        end
    end
    
    methods (Static)
        function results = compare(DP,polyMap,param)
            % Function which compares the DPs (generated from the odometry
            % data) with the polygonal map+
            %
            % TODO: Generalize this such that it can be used for both the
            % mapping and localization
            %
            % Input:
            %   DP:         Dominant points
            %   polyMap:    poly map representation
            %   param:      parameter
            %                   c_min
            %                   c_diff
            % Output:
            %   result:     structure with results
            
            [phiCum_DP, LCum_DP] = getCumOrientation(DP);
            DP_map = [polyMap.x polyMap.x(2:end); polyMap.y polyMap.y(2:end)];
            m = length(polyMap.x) - 1;
            corr = zeros(m,1);
            for j=1:1:m
                [phiCum_map, LCum_map] = getCumOrientation(DP_map(:,1:(m+j)));
                corr(j) = getCorrelation(phiCum_DP,LCum_DP,phiCum_map,LCum_map);
            end
            [corr_min,corr_idx] = min(corr);
            corr_tmp = corr;
            corr_tmp(corr_idx) = [];
            if ((min(corr_tmp) - corr_min) > param.c_diff) && (corr_min < param.c_min)
                if corr_idx == 1
                    corr_idx = length(polyMap.x);
                end
                results.corr = corr;     % Debug
                results.foundPosition = true;
                results.estPose = [polyMap.x(corr_idx); polyMap.y(corr_idx)];
                results.estPose(3) = atan2(polyMap.y(corr_idx)-polyMap.y(corr_idx-1), ...
                                    polyMap.x(corr_idx)-polyMap.x(corr_idx-1));
            else
                results.foundPosition = false;
                results.corr = corr;
            end
            
            % Internal functions
            function [phiCum, LCum] = getCumOrientation(DP)
                % Calculate the orientation (accumulated) in regard to the travelled path, 
                % therefore we create a piecewise linear function
                % 
                % input:
                %   DP: Dominant Points
                % output:
                %   phiCum: Cumulated angles
                %   LCum:   Cumulated lengths
                n = length(DP(1,:));      	% Number of Dominant Points
                phi = zeros(n-1,1);
                L = zeros(n-1,1);
                for i=n:-1:2                        % Go through all DPs
                    v = DP(:,i-1) - DP(:,i);        % TODO: Esnure backward is correct
                    phi(1+n-i) = atan2(v(2),v(1));  % Orientation of line segments
                    L(1+n-i) = norm(v);             % Length of line segments
                end
                % Cumulate Orientations
                phiCum = zeros(n-1,1);
                LCum = zeros(n-1,1);
                LCum(1) = L(1);
                for i = 2:1:(n-1)
                  delta_phi = phi(i) - phi(i-1);
                    if abs(delta_phi) > pi
                      if phi(i-1) > 0.0
                        delta_phi = 2*pi + delta_phi;
                      else
                        delta_phi = delta_phi - 2*pi;
                      end
                    end
                  phiCum(i) = phiCum(i-1) + delta_phi;  
                  LCum(i) = LCum(i-1) + L(i);
                end
            end
            function corr = getCorrelation(phiCum1,LCum1,phiCum2,LCum2)
                % Calculate correlation
                n = 1000;
                corr = 0;
                Lmax = min([LCum1(end),LCum2(end)]);
                l_add = linspace(0,Lmax,n);
                for k=1:1:n
                    phi1 = phiCum1(end);
                    phi2 = phiCum2(end);
                    for i=length(LCum1):-1:1
                        if l_add(k) < LCum1(i)
                            phi1 = phiCum1(i);
                        end
                    end
                    for i=length(LCum2):-1:1
                        if l_add(k) < LCum2(i)
                            phi2 = phiCum2(i);
                        end
                    end
                    corr = corr + (phi1 - phi2)^2;
                end
                corr = corr / n;
            end
        end 
    end
end
