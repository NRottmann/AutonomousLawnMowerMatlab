classdef PoseGraphOptimization
    % This class is used for pose graph optimization as presented in (1).
    % Please refer to the manual.
    %
    % Methods:
    %  	PoseGraphOptimization(pathData)
    %       This is tne constructot of the class
    
    % Nils Rottmann (Nils.Rottmann@rob.uni-luebeck.de)
    % 24.01.2018
    
    properties
        PathData;       % Data retieved by driving with the vehicle along the boundary line
        
        L_min;          % Parameter for data pruning
        E_max;
        L_nh;           % Parameter for correlation calculation
        C_min;
        M;
        Gamma1;
        Gamma2;
    end
    
    methods
        function obj = PoseGraphOptimization()
            % This is the constructor of the class

            % Get Parameters
            out = get_config('mapping');
            obj.L_min = out.l_min;
            obj.E_max = out.e_max;
            obj.L_nh = out.l_nh;
            obj.C_min = out.c_min;
            obj.M = out.M;
            obj.Gamma1 = out.gamma1;
            obj.Gamma2 = out.gamma2;
        end
        
        function [X,A] = generateMap(obj,pathData,optimize)
            % This is the main function of the class which runs the mapping
            % algorithm and gives back an map estimate
            %
            % Syntax:
            %       [] = generateMap(obj)
            %
            % Input:
            %   obj:        Object of the this class
            %   pathData:   The path data required for mapping
            %   optimize:   struct for optimization
            %       loopClosure:    if true, we optimize l_nh and c_min
            %       mapping:        if true, we optimize gamma1 and gamma2
            %
            % Output:
            %   X:      Pose Graph
            %   A:      Incidence matrix
            %
            
            % (0) Allocate path data
            % Check size, matrix has to be 2 x N
            if ~(length(pathData(:,1)) == 2)
                error('PoseGraphOptimization: Size of path data incorrect, should be 2 x N!')
            end
            obj.PathData = pathData;
            
            % (1) Data Pruning of the odometry data
            disp(['Prune ',num2str(length(obj.PathData(1,:))),' data points ...'])
            pruningParam.e_max = obj.E_max;
            pruningParam.l_min = obj.L_min;
            DP = generateDPs(obj.PathData,pruningParam);
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
            [SP,corr,L,optimParam] = PoseGraphOptimization.generateSPs(DP,correlationParam,optimize.loopClosure);
            obj.L_nh = optimParam.l_nh;
            obj.C_min = optimParam.c_min;
            disp(['Found ',num2str((sum(sum(SP))-length(SP(1,:)))),' similar pairs!'])
            
%             % Matlab solution
%             disp('PGO using Matlab library ...')
%             matlabGraph = PoseGraphOptimization.matlabPGO(xi,SP);
            
            % Tutorial Solution
            disp('PGO using tutorial approach ...')
            optParam.gamma1 = obj.Gamma1;
            optParam.gamma2 = obj.Gamma2;
            [X,A,optimParam] = PoseGraphOptimization.tutorialPGO(xi,SP,corr,L,optParam,optimize.mapping);
            obj.Gamma1 = optimParam.gamma1;
            obj.Gamma2 = optimParam.gamma2;
            
%             % Adjusted Tutorial Solution
%             disp('PGO using adjusted tutorial approach ...')
%             adjustedTutorialGraph = PoseGraphOptimization.tutorialPGOAdjusted(xi,SP);
            
%             % LAGO
%             disp('PGO using LAGO ...')
%             lagoGraph = PoseGraphOptimization.lagoPGO(xi,SP);
        end       
    end
    
    methods (Static)   
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
                xi(1:2,i-1) = R' * (data(:,i+1) - data(:,i));
                % Regularization, TODO: Check if this is valid
                xi(3,i-1) = theta(i) - theta(i-1);
                if xi(3,i-1) > pi
                    xi(3,i-1) = xi(3,i-1) - 2*pi;
                elseif xi(3,i-1) < -pi
                    xi(3,i-1) = xi(3,i-1) + 2*pi;
                end              
            end
        end
        
         function X = generatePoses(xi,x0)
            % Function to generate the poses from the given measurements of
            % the odometry
            %
            % input:
            %   xi:     measurements of the odometry (relative poses)
            %   x0:     Starting pose
            % output:
            %   X:      Poses
            
            % TODO: Size check
            
            N = length(xi(1,:));
            X = zeros(3,N+1);
            X(:,1) = x0;
            for i=1:1:N
                R = [cos(X(3,i)), -sin(X(3,i)); ...
                                sin(X(3,i)), cos(X(3,i))];
                X(1:2,i+1) = X(1:2,i) + R*xi(1:2,i);
                X(3,i+1) = X(3,i) + xi(3,i);
            end   
        end
        
        function [SP,corr,L,optimParam] = generateSPs(data,param,optimize)
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
            
            % Decide wether a parameter optimization is required or not
            if ~optimize
                % Calculate similar points
                [SP,L] = calculateSP(param);
                optimParam = param;
            else
                % Bayes Optimization
                l_nh = optimizableVariable('l_nh',[1,30]);
                c_min = optimizableVariable('c_min',[0.01,10],'Transform','log');
                thetaOpt = [l_nh,c_min];
                results = bayesopt(@loopClosureCost,thetaOpt);
                % Calculate optimized similar points
                [SP,L] = calculateSP(results.XAtMinObjective);
                optimParam = results.XAtMinObjective;
            end
            
            % Define cost function for Bayesian Optimization
            function cost = loopClosureCost(theta)
                [~,U] = calculateSP(theta);
                cost = inf;
                for ll = 1:1:round(length(U)/10)
                    GMModel = fitgmdist(U,ll,'RegularizationValue',0.1);
                    newcost = GMModel.NegativeLogLikelihood + (1000/length(U));
                    diff = cost - newcost;
                    if diff < 1
                        break;
                    else
                        cost = newcost;
                    end
                end
            end
            function [SP,L] = calculateSP(theta)
                l_min = theta.l_nh;                         % Minimum Length
                l_max = l_cumulated(end) - theta.l_nh;      % Maximum Length
                SP = zeros(M);
                L = [];                                     % length between the loop closing points
                for ii=1:1:M
                    if l_cumulated(ii) > l_min && l_cumulated(ii) < l_max
                        [pks, locs] = findpeaks(-corr(ii,:));
                        locs(abs(pks) > theta.c_min) = [];
                        SP(ii,locs) = 1;
                        for jj=1:1:length(locs)
                            if locs(jj) ~= ii
                                L = [L; abs(l_cumulated(ii) - l_cumulated(locs(jj)))];
                            end
                        end
                    end
                end
            end
        end
        
        function graph = matlabPGO(xi,SP)
            % Optimizes the pose graph using the method which is already
            % implemented in Matlab
            % 
            % input:
            %   xi:     measurements from the odometry (the orientations are
            %           already regulized)
            %   SP:     Matrix which contains informations about loop closures
            %   
            % output:
            %   graph:  Estimation of new pose graph nodes   
            %
            
            % Generate pose graph
            % TODO: Add correct covariances!
            disp(['Generate pose graph ...'])
            poseGraph = robotics.PoseGraph;
            for i=1:1:length(xi(1,:))
                poseGraph.addRelativePose(xi(:,i)');
            end
            % Add loop closures
            disp(['Add loop closures ...'])
            for i=2:1:length(SP(1,:))-1
                for j=(1+i):1:length(SP(1,:))-1
                    if SP(i,j) == 1
                        % TODO: Which kind of information matrix is
                        % required?
                        poseGraph.addRelativePose([0 0 0], [1 0 0 1 0 1], ...
                                                i-1,j-1);
                    end
                end
            end
            % Optimize pose graph
            disp(['Optimize pose graph ...'])
            graph = optimizePoseGraph(poseGraph);
        end
        
        function [X_opt,A,optimParam] = tutorialPGO(xi,SP,corr,L,param,optimize)
            % Optimizes the pose graph using the method presented in (1)
            %
            % (1) A tutorial on graph-based SLAM
            % 
            % input:
            %   xi:     measurements from the odometry (the orientations are
            %           already regulized)
            %   SP:     Matrix which contains informations about loop closures
            %   corr:   Correlation matrix according to the SPs
            %   
            % output:
            %   X:   	Estimation of new pose graph nodes   
            %   A:      Incidence Matrix
            
            % check sizes
            if (length(xi(1,:)) + 2) ~= length(SP(1,:))
                error('Sizes between xi and SP are not correct!')
            end
            
            % Generate the reduced incidence matrix for the odometry 
            % measurements (without the starting point)
            disp(['Generate incidence matrix for odometry measurements ...'])
            N = length(xi(1,:));
            A = diag(-1*ones(N,1)) + diag(ones(N-1,1),-1);
            A = [A; [zeros(1,N-1), 1]];
            
            % Add loop closure constraints
            % TODO: is this correct?
            disp(['Add loop closures ...'])
            C = [];
            for i=1:1:(N+1)
                for j=(1+i):1:(N+1)
                    % If there is a connection we add a column to the
                    % incidence matrix
                    if SP(i,j) == 1
                        v = zeros(N+1,1);
                        v(i) = -1; v(j) = 1;
                        A = [A, v];
                        C = [C, corr(i,j)];
                    end
                end
            end
            
            % Add measurements for the loop closure. Here we assume that
            % the difference in distance and orientation are zero
            M = length(A(1,:)) - N;
            xi_lc = [xi, zeros(3,M)];
            
            % Generate initial guess of the poses using the odometry
            % measurements with [0,0,0]^T as starting point
            X = PoseGraphOptimization.generatePoses(xi,[0;0;0]);
            
            % Define information gain matrices
            % TODO: Do something better then everywhere simply the identity
            % matrix
            Omega = cell(N+M,1);
        
            
            out = get_config('odometryModelNoise');
            for i=1:1:N         % Odometric constraints
                deltaT = norm(xi_lc(1:2,i));
                deltaR = abs(xi_lc(3,i));
                sigma(1) = abs(cos(X(3,i)) * (out.a(3)*deltaT + out.a(4)*deltaR));
                sigma(2) = abs(sin(X(3,i)) * (out.a(3)*deltaT + out.a(4)*deltaR));
                sigma(3) = (out.a(1)*deltaR + out.a(2)*deltaT);
                for j=1:1:3     % Avoid singularities
                    if sigma(j) < 10^(-3)
                        sigma(j) = 10^(-3);
                    end
                end
                Omega{i} = diag(sigma)^(-1);
            end
            
            % Decide wether we optimize mapping parameter
            if ~optimize
                % Optimize the path data
                [X_opt] = getOptimizedPath(param);
                optimParam = param;
            else
                % Calculate the circumference
                logLikelihood = inf;
                GMModelOld = [];
                for ll = 1:1:round(length(L)/10)
                    GMModel = fitgmdist(L,ll,'RegularizationValue',0.1);
                    diff = logLikelihood - GMModel.NegativeLogLikelihood;
                    if diff < 1
                        break;
                    else
                        logLikelihood = GMModel.NegativeLogLikelihood;
                        GMModelOld = GMModel;
                    end
                end
                Circumference = min(GMModelOld.mu);
                Cluster = GMModelOld.NumComponents;
                
                % Optimize parameters using Bayesian Optimization
                gamma1 = optimizableVariable('gamma1',[0.01,100],'Transform','log');
                gamma2 = optimizableVariable('gamma2',[0.01,100],'Transform','log');
                thetaOpt = [gamma1,gamma2];
                results = bayesopt(@PGOCost,thetaOpt);
                % Calculate optimized similar points
                [X_opt] = getOptimizedPath(results.XAtMinObjective);
                optimParam = results.XAtMinObjective;
            end
                 
            function cost = PGOCost(theta)
                [X_tmp] = getOptimizedPath(theta);
                % Go through all loop closures and calculate lengths
                U = zeros(M,1);
                for ii=N+1:1:N+M
                    [~,idx_min] = min(A(:,ii));
                    [~,idx_max] = max(A(:,ii));
                    for jj=1:1:idx_max-idx_min
                        U(ii-N) = U(ii-N) + norm(X_tmp(1:2,idx_min+jj) - X_tmp(1:2,idx_min+(jj-1)));
                    end
                end
                % Use mixture Models to get estimated circumference
                GMModel_tmp = fitgmdist(U,Cluster,'RegularizationValue',0.1);
                U_mean = min(GMModel_tmp.mu);
                % Calculate cost
                cost = abs(Circumference - U_mean);
            end
                  
            function [X_opt] = getOptimizedPath(theta)
                % Adjust loop closing constraints according to the given
                % parameters
                for i=N+1:1:N+M   	% Loop closing constraints
                    Omega{i} = diag([1/theta.gamma1 1/theta.gamma1 1/theta.gamma2]) * (1/C(i-N));
                end
                % Compute Hessian and coefficient vector
                e = inf;
                count = 0;
                while (e > 0.001) && (count < 100)
                    count = count + 1;
                    b = zeros(3*(N+1),1);
                    H = zeros(3*(N+1));
                    for ii=1:1:(N+M)
                        [~,i] = min(A(:,ii));
                        [~,j] = max(A(:,ii));
                        [AA,BB] = PoseGraphOptimization.jacobianTutorial(xi_lc(:,ii),X(:,i),X(:,j));
                        i1 = 3*i-2; i2 = 3*i;
                        j1 = 3*j-2; j2 = 3*j;
                        % Hessians
                        H(i1:i2,i1:i2) = H(i1:i2,i1:i2) + AA'*Omega{ii}*AA;
                        H(i1:i2,j1:j2) = H(i1:i2,j1:j2) + AA'*Omega{ii}*BB;
                        H(j1:j2,i1:i2) = H(j1:j2,i1:i2) + BB'*Omega{ii}*AA;
                        H(j1:j2,j1:j2) = H(j1:j2,j1:j2) + BB'*Omega{ii}*BB;
                        % coefficient vector
                        e_ij = PoseGraphOptimization.errorTutorial(xi_lc(:,ii),X(:,i),X(:,j));
                        b(i1:i2) = b(i1:i2) + AA'*Omega{ii}*e_ij;
                        b(j1:j2) = b(j1:j2) + BB'*Omega{ii}*e_ij;
                    end
                    % Keep first node fixed
                    H(1:3,1:3) = H(1:3,1:3) + eye(3);
                    % Solve the linear system
                    dX_tmp = (H \ (-b));
                    e = norm(dX_tmp);
                    % Update poses
                    dX = zeros(3,N+1);
                    for i=1:1:(N+1)
                        dX(:,i) = dX_tmp(((i*3)-2):(i*3));
                    end
                    X = X + dX;
                end 
                X_opt = X;
            end
        end
        
        function X_new = tutorialPGOAdjusted(xi,SP)
            % Optimizes the pose graph using the method presented in (1) by
            % adding measurements in a different way (TODO: explain how)
            %
            % (1) A tutorial on graph-based SLAM
            % 
            % input:
            %   xi:     measurements from the odometry (the orientations are
            %           already regulized)
            %   SP:     Matrix which contains informations about loop closures
            %   
            % output:
            %   p_est:  Estimation of new pose graph nodes   
            %
            
            % check sizes
            if (length(xi(1,:)) + 2) ~= length(SP(1,:))
                error('Sizes between xi and SP are not correct!')
            end
            
            % Generate the reduced incidence matrix for the odometry 
            % measurements (without the starting point)
            disp('Generate incidence matrix for odometry measurements ...')
            N = length(xi(1,:));
            A1 = diag(-1*ones(N,1)) + diag(ones(N-1,1),-1);
            A1 = [A1; [zeros(1,N-1), 1]];
            
            % Add loop closure constraints
            % TODO: is this correct?
            disp('Add loop closures ...')
            A2 = [];
            for i=1:1:(N+1)
                for j=(1+i):1:(N+1)
                    % If there is a connection we add a column to the
                    % incidence matrix
                    if SP(i,j) == 1
                        v = zeros(N+1,1);
                        v(i) = -1; v(j) = +1;
                        A2 = [A2, v];
                    end
                end
            end           
            M = length(A2(1,:));
            DP_indices = 1:1:(N+1);
            for i=1:1:M
                [~,idx1] = min(A2(:,i));
                [~,idx2] = max(A2(:,i));
                DP_indices(idx2) = DP_indices(idx1);
            end
            
            % Generate initial guess of the poses using the odometry
            % measurements with [0,0,0]^T as starting point
            X = PoseGraphOptimization.generatePoses(xi,[0;0;0]);
            
            % Reorder incidence Matrix for odometry measurements according
            % to loop closures
            indices = [];
            for i=1:1:(N+1)
                if i ~= DP_indices(i)
                    A1(DP_indices(i),:) = A1(DP_indices(i),:) + A1(i,:);
                    indices = [indices i];
                end
            end
            X(:,indices) = [];
            A1(indices,:) = [];
            K = length(A1(:,1));

            % Define information gain matrices
            % TODO: Do something better then everywhere simply the identity
            % matrix
            Omega = [1 0 0; 0 1 0; 0 0 10];
            
            % Compute Hessian and coefficient vector
            e = inf;
            count = 0;
            while (e > 0.001) && (count < 100)
                count = count + 1;
                b = zeros(3*K,1);
                H = zeros(3*K);
                for ii=1:1:N
                    [~,i] = min(A1(:,ii));
                    [~,j] = max(A1(:,ii));
                    [AA,BB] = PoseGraphOptimization.jacobianTutorial(xi(:,ii),X(:,i),X(:,j));
                    i1 = 3*i-2; i2 = 3*i;
                    j1 = 3*j-2; j2 = 3*j;
                    % Hessians
                    H(i1:i2,i1:i2) = H(i1:i2,i1:i2) + AA'*Omega*AA;
                    H(i1:i2,j1:j2) = H(i1:i2,j1:j2) + AA'*Omega*BB;
                    H(j1:j2,i1:i2) = H(j1:j2,i1:i2) + BB'*Omega*AA;
                    H(j1:j2,j1:j2) = H(j1:j2,j1:j2) + BB'*Omega*BB;
                    % Coefficient vector
                    e_ij = PoseGraphOptimization.errorTutorial(xi(:,ii),X(:,i),X(:,j));
                    b(i1:i2) = b(i1:i2) + AA'*Omega*e_ij;
                    b(j1:j2) = b(j1:j2) + BB'*Omega*e_ij;
                end
                % Keep first node fixed
                H(1:3,1:3) = H(1:3,1:3) + eye(3);
                % Solve the linear system
                dX_tmp = (H \ (-b));
                e = norm(dX_tmp);
                % Update poses
                dX = zeros(3,K);
                for i=1:1:K
                    dX(:,i) = dX_tmp(((i*3)-2):(i*3));
                end
                X = X + dX;
            end
            % Take only the first circle of the solution, TODO: Make this
            % more general or mathematics
            for i=1:1:(N+1)
                if i ~= DP_indices(i)
                    X_new = X(:,DP_indices(i):(i-1));
                    X_new = [X_new, X(:,DP_indices(i))];
                    break
                end
            end
        end
        
        function [A,B] = jacobianTutorial(z,xi,xj)
            % Calculates the Jacobian A and B
            %
            % input:
            %   z:      Measurement
            %   xi:     Pose i
            %   xj:     Pose j
            %
            % output:
            %   A,B:    Jacobians
            %
            
            dx = 10^(-9);
            
            e = PoseGraphOptimization.errorTutorial(z,xi,xj);
            A = zeros(3);
            B = zeros(3);
            for i=1:1:3
                xi_tmp = xi; xi_tmp(i) = xi_tmp(i) + dx;
                xj_tmp = xj; xj_tmp(i) = xj_tmp(i) + dx;
                A(:,i) = ((PoseGraphOptimization.errorTutorial(z,xi_tmp,xj)) - e) / dx;
                B(:,i) = ((PoseGraphOptimization.errorTutorial(z,xi,xj_tmp)) - e) / dx;
            end
        end
        
        function e = errorTutorial(z,xi,xj)
            % TODO: Add desciption
            R = [cos(xi(3)), -sin(xi(3)); ...
                            sin(xi(3)), cos(xi(3))];
            z_star = [R' * (xj(1:2) - xi(1:2)); xj(3)-xi(3)];
            % Regularization, TODO: Check if this is valid
            z_star(3) = z_star(3) - floor(z_star(3)/(2*pi))*2*pi;
            if z_star(3) > pi
                z_star(3) = z_star(3) - 2*pi;
            elseif z_star(3) < -pi
                z_star(3) = z_star(3) + 2*pi;
            end
            e = z - z_star;
        end
        
        function p_est = lagoPGO(xi,SP)
            % Optimizes the pose graph using the LAGo method*
            %
            % *A fast an accurate approximation for planar pose graph
            % optimization
            % 
            % input:
            %   xi:     measurements from the odometry (the orientations are
            %           already regulized)
            %   SP:     Matrix which contains informations about loop closures
            %   
            % output:
            %   p_est:  Estimation of new pose graph nodes   
            %
            
            % check sizes
            if (length(xi(1,:)) + 2) ~= length(SP(1,:))
                error('Sizes between xi and SP are not correct!')
            end
            
            % Generate the reduced incidence matrix for the odometry 
            % measurements (without the starting point)
            disp(['Generate reduced incidence matrix for odometry measurements ...'])
            N = length(xi(1,:));
            A = diag(ones(N,1)) + diag(-1*ones(N-1,1),1);
            
            % Add loop closure constraints
            disp(['Add loop closures ...'])
            for i=2:1:N
                for j=(1+i):1:N
                    % If there is a connection we add a column to the
                    % incidence matrix
                    if SP(i,j) == 1
                        v = zeros(N,1);
                        v(i-1) = -1; v(j-1) = 1;
                        A = [A, v];
                    end
                end
            end
            
            % Add measurements for the loop closure. Here we assume that
            % the difference in distance and orientation are zero
            M = length(A(1,:)) - N;
            xi_lc = [xi, zeros(3,M)];
            
            % Stacking the relative pose measurements
            delta_theta = zeros(N+M,1);
            delta_l = zeros(2*(N+M),1);
            for i=1:1:(N+M)
                delta_theta(i) = xi_lc(3,i);
                delta_l((2*i-1):(2*i)) = xi_lc(1:2,i);
            end
            
            % Generate information matrix
            % TODO: Get the covariances for the odometry measurements,
            % right now we assume all ones
            Info_delta = eye(N+M);
            Info_DeltaL = eye(2*(N+M));
            
            % PHASE 1 
            % Get the first orientation estimate
            disp('Calculate linear orientation estimate ...')
            Info_theta = (A*Info_delta*A');
            theta_linear = Info_theta \  (A*Info_delta*delta_theta);
            
            % PHASE 2
            % Generate rotation matrix
            R = PoseGraphOptimization.rotationMatrix(A,theta_linear);
            % Build Informationmatrix for poses
            Info_g = R*Info_DeltaL*R';
            J = PoseGraphOptimization.jacobianMatrix(A,delta_l,theta_linear);
            Info_z = [Info_g, -Info_g*J; -J'*Info_g, Info_theta+J'*Info_g*J];
            z = [(R*delta_l); theta_linear];
            
            % PHASE 3
            % Generate expandend incidence matrix
            A2 = kron(A,eye(2));
            B = [A2 zeros(2*N,N); zeros(N,2*(N+M)), eye(N)];
            Info_p = B*Info_z*B';
            p_tmp = Info_p \ (B*Info_z*z);
            
            % Allocate estimate
            p_est = zeros(3,N);
            for i=1:1:N
                p_est(1:2,i) = p_tmp((2*i-1):(2*i));
                p_est(3,i) = p_tmp(2*N+i);
            end
        end
        
        function R = rotationMatrix(A,theta)
            % Generates a rotation matrix required for the LAGO algorithm
            % 
            % input:
            %   A:      Reduced incidence matrix
            %   theta:  Angles theta
            %   
            % output:
            %   R:      Rotation matrix
            %   
            
            NM = length(A(1,:));
            R = zeros(2*NM);
            for i=1:1:NM
                [~,idx] = min(A(:,i));
                R_tmp = [cos(theta(idx)), -sin(theta(idx)); ...
                                sin(theta(idx)), cos(theta(idx))];
                R((2*i-1):(2*i),(2*i-1):(2*i)) = R_tmp; 
            end
        end
        
        function J = jacobianMatrix(A,delta_l,theta)
            % Generates a jacobian matrix required for the LAGO algorithm.
            % Here we calulate the Jacobian numerically
            % 
            % input:
            %   A:          Reduced incidence matrix
            %   delta_l:    relative position measurements     
            %   theta:      Angles
            %   
            % output:
            %   J:      	Jacobian in regard to the theta's
            %  
            
            % Jacobian step size
            dtheta = 10^(-12);
            
            % Calculate Jacobian
            N = length(theta);
            M2 = length(delta_l);
            J = zeros(M2,N);
            for i=1:1:N
                theta_tmp = theta;
                theta_tmp(i) = theta_tmp(i) + dtheta;
                J(:,i) = (PoseGraphOptimization.rotationMatrix(A,theta_tmp)*delta_l ...
                            - PoseGraphOptimization.rotationMatrix(A,theta)*delta_l) / dtheta;
            end            
        end
    end
end

