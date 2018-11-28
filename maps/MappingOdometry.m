classdef MappingOdometry
    % This is the mapping approach as presented in the rejected paper to
    % icra
    %
    % Mapping class for autonomous lawn mower with odometry only data
    % Methods:
    %  ...
    
    % Nils Rottmann (Nils.Rottmann@rob.uni-luebeck.de)
    % 05.10.2018
    
    properties
        OdoData;
        Lmin;
        Emax;
        Lnh;
        Cmin;
        M;
        Alpha;
        Beta;
        Gamma;
        MaxIter;
        MinDiff;
    end
    
    methods
        function obj = MappingOdometry(odoData)
            % This is the constructor of the class
            % Syntax:
            %       obj = MappingOdometry(odometryData)
            
            % Output:
            %   obj:            Intance of the class MappingOdometry
            
            obj.OdoData = odoData;
            
            % Get Parameters
            out = get_config('mapping');
            obj.Lmin = out.L_min;
            obj.Emax = out.e_max;
            obj.Lnh = out.L_nh;
            obj.Cmin = out.C_min;
            obj.M = out.M;
            obj.Alpha = out.alpha;
            obj.Beta = out.beta;
            obj.Gamma = out.gamma;
            obj.MaxIter = out.maxIter;
            obj.MinDiff = out.minDiff;
        end
        
        function [polyMap,fittingPairs,DP,DP_opt] = map(obj)
            % This is the main function of the class which runs the mapping
            % algorithm
            % Syntax:
            %       polyMap = map(obj)
            % Input:
            %   obj:            Object of the mapping class
            %
            % Output:
            %   polyMap:       	A polyMap
            
            %% Line Segmentation
            num = length(obj.OdoData(1,:));
            X = ['Prune ',num2str(num),' data points ...'];
            disp(X)
            DP = MappingOdometry.lineSegmentation(obj.OdoData,obj.Lmin,obj.Emax);
            num = length(DP(1,:));
            X = ['Data points number reduced to ',num2str(num),'!'];
            disp(X)
            
            %% Calculate the orientation (accumulated) and plot results
            disp('Generate orientation function ...')
            [phiCum, LCum] = MappingOdometry.getCumOrientation(DP);
            disp('Orientation function successfully generated!')
            
            %% Get a correlation between dominant points and find similar ones
            disp('Find Correlations ...')
            [~,fittingPairs] = MappingOdometry.getCorrelation(phiCum,LCum,obj.Lnh,obj.Cmin,obj.M);
            num = length(fittingPairs);
            X = ['Found ',num2str(num),' similarity pairs!'];
            disp(X)
            
            %% Close the loop using our optiization method
            disp('Optimize DPs ...')
            [vs_base, vss_base] = MappingOdometry.energy(DP);
            [DP_opt,DP_cl,~,~] = MappingOdometry.loopClosure(DP,vs_base,vss_base,fittingPairs,...
                                                    obj.Alpha,obj.Beta,obj.Gamma,obj.MaxIter,obj.MinDiff);
            disp('Optimization completed! Generate map!')                                  
            
            %% Generate polymap
            polyMap.x = DP_cl(1,:);
            polyMap.y = DP_cl(2,:);
            polyMap.XMapLimits = [min(polyMap.x); max(polyMap.x)];
            polyMap.YMapLimits = [min(polyMap.y); max(polyMap.y)];
            polyMap.XWorldLimits = [polyMap.XMapLimits(1)-1; polyMap.XMapLimits(2)+1];
            polyMap.YWorldLimits = [polyMap.YMapLimits(1)-1; polyMap.YMapLimits(2)+1];
        end       
    end
    
    methods (Static)  
        function DP = lineSegmentation(odoData,L_min,e_max)
            % Here we try to part the estimated path into line segments in order to
            % define a map structure
            %
            % input:
            %   obj: instance of the class
            %   L_min: Minimum distance between first and last point of a segment
            %   e_max: Error for segmentation
            % output:
            %   DP: Dominant Points which apprx. the path by connection of straight
            %   lines between them

            n = length(odoData(1,:));
            x_0 = odoData(:,1);                     % Starting point of line segment
            X = x_0;                                % Data points of actual line segment
            DP = x_0;

            for i = 2:1:n
                % Calculate distance between actual point and actual arc origin
                dis = norm(x_0 - odoData(:,i));
                % if distance is smaller a treshold, simply add the point to the actual
                % segment, else go on
                if (dis < L_min)
                    X = [X odoData(:,i)];
                else
                    X_tmp = [X odoData(:,i)];
                    % Get the error for the current data
                    e = errorLineFit(X_tmp);
                    if e < e_max    % error is small enough, align data
                        X = X_tmp;
                    else 
                        x_0 = X(:,end);
                        DP = [DP x_0]; 
                        X = [x_0 odoData(:,i)];
                    end
                end 
            end
            DP = [DP odoData(:,end)];

            % Function to define the error of the line fit to the pathData
            function e = errorLineFit(X)
                x0 = X(:,1);
                x1 = X(:,end);
                v = x1 - x0;
                phi = atan2(v(2),v(1));
                m = length(X(1,:));
                e = 0;
                for j=1:1:m
                    e = e + norm(X(:,j) - getXLine(X(:,j),x0,phi))^2;
                end
                e = e/m;
            end
            function x_line = getXLine(x,x0,phi)
                phi_star = atan2(x(2)-x0(2),x(1)-x0(1));
                d = norm(x - x0);
                psi = phi - phi_star;
                l = d/cos(psi);
                x_line = x0 + [cos(phi); sin(phi)]*l;
            end

        end 
        
        function [phiCum, LCum] = getCumOrientation(DP)
            % Calculate the orientation (accumulated) in regard to the travelled path, 
            % therefore we create a piecewise linear function
            % 
            % input:
            %   DP: Dominant Points
            % output:
            %   phiCum: Cumulated angles
            %   LCum:   Cumulated lengths

            n_DP = length(DP(1,:));             % Number of Dominant Points
            phi = zeros(n_DP,1);
            L = zeros(n_DP,1);
            for i=3:1:n_DP                      % Go through all DPs
                v = DP(:,i) - DP(:,i-1);
                phi_tmp = atan2(v(2),v(1));
                phi(i-1) = phi_tmp;                        % Orientation of line segments
                L(i-1) = norm(DP(:,i) - DP(:,i-1));        % Length of line segments
            end
            % Cumulate Orientations
            phiCum = zeros(n_DP,1);
            LCum = zeros(n_DP,1);
            for i = 2:1:n_DP
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

        function [corr,fittingPairs] = getCorrelation(phiCum,LCum,L_max,c_min,M)
            % Calculate the correlation between the dominant points
            %
            % input:
            %   phiCum: Cumulated orientations
            %   LCum:   Cumulated lengths
            %   L_max:  Neighborhood parameter
            %   c_min:  minimal correlation
            % output:
            %   corr:   correlation matrix
            %   fittingPairs:   Pairs of DPs which are similar

            % Calculate correlation
            n_DP = length(phiCum);
            n = M;                    % M points for comparison
            corr = zeros(n_DP);
            l_add = linspace(-L_max,L_max,n);
            for i=1:1:n_DP
                LCum_i = LCum - LCum(i);
                phiCum_i = phiCum - phiCum(i);
                for j=1:1:n_DP
                    LCum_j = LCum - LCum(j);
                    phiCum_j = phiCum - phiCum(j);
                    for k=1:1:n
                        corr(i,j) = corr(i,j) ...
                            + (MappingOdometry.getOrientation(phiCum_i,LCum_i,l_add(k)) ...
                                    - MappingOdometry.getOrientation(phiCum_j,LCum_j,l_add(k)))^2;
                    end
                    corr(i,j) = corr(i,j) / n;
                end
            end

            % Calculate similar points
            L_min = L_max;                 	% Minimum Length
            L_max = LCum(end) - L_max;         % Maximum Length
            fittingPairs = cell(n_DP,1);
            for i=1:1:n_DP
                if LCum(i) > L_min && LCum(i) < L_max
                    [pks, locs] = findpeaks(-corr(i,:));
                    locs(abs(pks) > c_min) = [];
                    locs(locs == i) = [];
                    fittingPairs{i,1} = [i locs];
                end
            end
      	end
             
       function [DP_opt,DP_cl,s1,s2] = loopClosure(DP,vs_base,vss_base,fittingPairs,alpha,beta,gamma,maxIter,minDiff)
            % Loop Closure using the proposed optimization method for odometry only
            % data
            %
            % input:
            %   DP: Dominant Points
            %   vs_base: Original length between DPs
            %   vss_base: Original angle between line segments
            %   fittingPairs: Pairs of DPs which fit to each other
            %   alpha, beta, gamma: Parameters
            %   maxIter: Maximum number of iterations for optimization process
            % output:
            %   DP_opt: Optimized DPs
            %   DP_cl: DPs for closed loop representation
            %   s1, s2: index of start and end DP for Loop Closure

            diff = 1;
            delta = 10^(-9);
            count = 0;
            while diff > minDiff && count < maxIter
                n = length(DP(1,:));
                dx = 10^(-12);
                f = MappingOdometry.internalEnergy(DP,vs_base,vss_base,alpha,beta) ...
                        + MappingOdometry.externalEnergy(DP,fittingPairs,gamma);
                J = zeros(n,2);
                for i=1:1:n
                    for j=1:1:2
                        % Adjust positions
                        DP_tmp = DP;
                        DP_tmp(j,i) = DP_tmp(j,i) + dx;
                        % Evaluate function
                        f_dx = MappingOdometry.internalEnergy(DP_tmp,vs_base,vss_base,alpha,beta) ...
                                + MappingOdometry.externalEnergy(DP_tmp,fittingPairs,gamma);
                        % Adjust changes accoridng to Newtons Method
                        J(i,j) = (f_dx - f) / dx;
                    end
                end
                Delta_F = J'*f;
                DP_tmp = DP - delta * Delta_F;
                f_new = MappingOdometry.internalEnergy(DP_tmp,vs_base,vss_base,alpha,beta) ...
                            + MappingOdometry.externalEnergy(DP_tmp,fittingPairs,gamma);
                diff = f - f_new;
                while diff < 0
                    delta = delta * 0.9;
                    DP_tmp = DP - delta * Delta_F;
                    f_new = MappingOdometry.internalEnergy(DP_tmp,vs_base,vss_base,alpha,beta) ...
                            + MappingOdometry.externalEnergy(DP_tmp,fittingPairs,gamma);
                    diff = f - f_new;
                end
                delta = delta * 2;
                DP = DP_tmp;
                count = count + 1;
            end  
            DP_opt = DP;

            % Close the loop
            % First find the best fitting Pair for loop closure
            nn = length(fittingPairs);
            D = ones(nn,1) * inf;
            for i=1:1:nn
                mm = length(fittingPairs{i});
                e = 0;
                if mm > 1
                    for j=2:1:mm
                        e = e + norm(DP_opt(:,fittingPairs{i}(1)) - DP_opt(:,fittingPairs{i}(j)));
                    end
                    D(i) = e/mm;
                end
            end
            [~,idx] = min(D);
            nn = length(fittingPairs{idx});
            D = zeros(nn-1,1);
            for i=2:1:nn
                D(i-1) = abs(fittingPairs{idx}(1) - fittingPairs{idx}(i));
            end
            [diff,idx2] = min(D);
            idx2 = idx2 + 1;
            if fittingPairs{idx}(1) > fittingPairs{idx}(idx2)
                s1 = fittingPairs{idx}(idx2);
                s2 = fittingPairs{idx}(1);
            else
                s1 = fittingPairs{idx}(1);
                s2 = fittingPairs{idx}(idx2);
            end
            % TODO: Check this failure out, added +1 behind diff, also try
            % to optimize this
            DP_cl = zeros(2,diff+1);
            for i=s1:1:s2-1
                x_mean = mean(DP_opt(1,fittingPairs{i}));
                y_mean = mean(DP_opt(2,fittingPairs{i}));
                DP_cl(:,i-s1+1) = [x_mean; y_mean];
            end
            DP_cl(:,end) = DP_cl(:,1);
       end
       
       function E_ext = externalEnergy(DP,fittingPairs,gamma)
            % Function for the calculation of the external energy
            %
            % Input:
            %   DP: Dominant Points
            %   fittingPairs: Indices which DPs fit together
            %   gamma: Parameter for external energy
            % Output:
            %   E_ext: External Energy

            n = length(DP(1,:)) - 1;

            E_ext = 0;
            c = 0;
            for i=1:1:n
                l = length(fittingPairs{i});
                if l > 1
                    for j=2:1:l
                        E_ext = E_ext + gamma * ...
                                norm(DP(:,fittingPairs{i}(1)) - DP(:,fittingPairs{i}(j)))^2;
                        c = c + 1;
                    end
                end
            end
            E_ext = E_ext / c;
       end
        
       function E_int = internalEnergy(DP,vs_base,vss_base,alpha,beta)
            % Function for the calculation of the internal energy
            %
            % input:
            %   DP: Actual Dominant Points
            %   vs_base: Length between DPs of the original odometry data
            %   vss_base: Change in orientation at DPs of the original odometry data
            %   alpha: Energy Paramter for length
            %   beta: Energy parameter for orientation change
            % output:
            %   E_int: Differecne in internal energy

            [vs, vss] = MappingOdometry.energy(DP);

            n = length(vs);

            E_int = 0;

            % Calculate angle differences
            dAngle = zeros(n,1);
            for i=1:1:n
                dAngle(i) = vss(i)-vss_base(i);
                if dAngle(i) < -pi
                    dAngle(i) = 2*pi + dAngle(i);
                elseif dAngle(i) > pi
                    dAngle(i) = dAngle(i) - 2*pi;
                end
                if vss_base(i) < -pi
                    vss_base(i) = 2*pi + vss_base(i);
                elseif vss_base(i) > pi
                    vss_base(i) = 2*pi - vss_base(i);
                end
            end
            for i=1:1:n
                E_int = E_int + alpha*((1/vs_base(i))*(vs(i)-vs_base(i)))^2 ...
                                        + beta*((1/vss_base(i))*dAngle(i))^2;                     
            end
            E_int = E_int / n;

            % Adding circumference
            E_int  = E_int + alpha*(1/sum(vs_base))*(sum(vs_base)-sum(vs))^2;
       end
       
       function [vs, vss] = energy(DP)
            % Calculates the length and change of orientation of the path segments
            %
            % Input:
            %   DP: Dominant Points
            % Output:
            %   vs: vector of length between DPs
            %   vss: vector of change in orientation at DPs
            n = length(DP(1,:)) - 1;
            vs = zeros(n,1);            % The length energy
            vss = vs;                   % The curve energy

            vs(1) = norm(DP(:,2) - DP(:,1));

            v1 = DP(:,end) - DP(:,end-1);
            v2 = DP(:,2) - DP(:,1);
            vss(1) = atan2(v2(2),v2(1)) - atan2(v1(2),v1(1));

            for i=2:1:n
                vs(i) = norm(DP(:,i+1) - DP(:,i));

                v1 = DP(:,i) - DP(:,i-1);
                v2 = DP(:,i+1) - DP(:,i);
                vss(i) = atan2(v2(2),v2(1)) - atan2(v1(2),v1(1));       
            end
       end
       
       function phi = getOrientation(phiCum,LCum,x)
            % Get the orientation based on the cumulative length and cumulative
            % orientation
            %
            % input:
            %   phiCum: Cumulative orientation
            %   LCum: Cumulative length
            %   x: Evalutation position
            % output:
            %   phi: The orientation at position x

            phi = 0;
            for i=1:1:length(LCum)-1
                if LCum(i) < x
                    phi = phiCum(i+1);
                end
            end
       end
    end
end

