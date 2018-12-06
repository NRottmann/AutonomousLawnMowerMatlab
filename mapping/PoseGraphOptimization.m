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
        
        function [matlabGraph,tutorialGraph,adjustedTutorialGraph,lagoGraph] = generateMap(obj)
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
            disp('PGO using Matlab library ...')
            matlabGraph = PoseGraphOptimization.matlabPGO(xi,SP);
            
            % Tutorial Solution
            disp('PGO using tutorial approach ...')
            tutorialGraph = PoseGraphOptimization.tutorialPGO(xi,SP);
            
            % Adjusted Tutorial Solution
            disp('PGO using adjusted tutorial approach ...')
            adjustedTutorialGraph = PoseGraphOptimization.tutorialPGOAdjusted(xi,SP);
            
            % LAGO
            disp('PGO using LAGO ...')
            lagoGraph = PoseGraphOptimization.lagoPGO(xi,SP);
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
        
        function X = tutorialPGO(xi,SP)
            % Optimizes the pose graph using the method presented in (1)
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
            disp(['Generate incidence matrix for odometry measurements ...'])
            N = length(xi(1,:));
            A = diag(-1*ones(N,1)) + diag(ones(N-1,1),-1);
            A = [A; [zeros(1,N-1), 1]];
            
            % Add loop closure constraints
            % TODO: is this correct?
            disp(['Add loop closures ...'])
            for i=1:1:(N+1)
                for j=(1+i):1:(N+1)
                    % If there is a connection we add a column to the
                    % incidence matrix
                    if SP(i,j) == 1
                        v = zeros(N+1,1);
                        v(i) = -1; v(j) = 1;
                        A = [A, v];
                    end
                end
            end
            
            % Add measurements for the loop closure. Here we assume that
            % the difference in distance and orientation are zero
            M = length(A(1,:)) - N;
            xi_lc = [xi, zeros(3,M)];
            
            % Define information gain matrices
            % TODO: Do something better then everywhere simply the identity
            % matrix
            Omega = [1 0 0; 0 1 0; 0 0 10];
            
            % Generate initial guess of the poses using the odometry
            % measurements with [0,0,0]^T as starting point
            X = PoseGraphOptimization.generatePoses(xi,[0;0;0]);
            
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
                    H(i1:i2,i1:i2) = H(i1:i2,i1:i2) + AA'*Omega*AA;
                    H(i1:i2,j1:j2) = H(i1:i2,j1:j2) + AA'*Omega*BB;
                    H(j1:j2,i1:i2) = H(j1:j2,i1:i2) + BB'*Omega*AA;
                    H(j1:j2,j1:j2) = H(j1:j2,j1:j2) + BB'*Omega*BB;
                    % coefficient vector
                    e_ij = PoseGraphOptimization.errorTutorial(xi_lc(:,ii),X(:,i),X(:,j));
                    b(i1:i2) = b(i1:i2) + AA'*Omega*e_ij;
                    b(j1:j2) = b(j1:j2) + BB'*Omega*e_ij;
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
            % Clear beginning and end
            [~,idx1] = min(A(:,N+2));
            [~,idx2] = max(A(:,end));
            X = X(:,idx1:idx2);
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

