classdef Optimization
    % This class is used for different optimization techniques
    %
    % Nils Rottmann (Nils.Rottmann@rob.uni-luebeck.de)
    % 11.03.2019
    
    properties
        % Classes
        ControlUnit;        % Instance of class control unit
    end
    
    methods
        function obj = Optimization(controlUnit)
            % Constructor
            obj.ControlUnit = controlUnit;
        end
        
        function results = BayesianOptimization(obj,estPath)
            % Bayesian Optimization
            controlUnit = obj.ControlUnit;
            
            % Evaluation Matrix
            x = [0.1:0.1:10];
            X_ = [];
            for i=1:1:length(x)
                X_ = [X_, [ones(1,length(x))*x(i); x]];
            end
            K_ss = Optimization.se_kernel(X_,X_);
            
            % Set initial parameters and calculate intitial guess
            X = [1; 1];
            controlUnit.ClassPoseGraphOptimization.Gamma1 = X(1);
            controlUnit.ClassPoseGraphOptimization.Gamma2 = X(2);        
            [controlUnit,~] = controlUnit.mapping(estPath);
            results = controlUnit.compare(4);
            f = -results.error;
            
            % Go over ten iterations
            for i=1:1:20
                % Calculate Covariances
                K_s = Optimization.se_kernel(X,X_);
                K = Optimization.se_kernel(X,X); 
                
                % Ensure positive definiteness
                K = 0.1*eye(length(K)) + K; 
                
                % Calculating mean vector and covariance matrix
                L = chol((K),'lower');    % Cholesky Tranformation, L satisfies L*L' = K
                alpha = (L')\(L\f);       % avoiding inverse transformation for computational effectiveness
                mu = (K_s')*alpha;
                beta = L \ K_s;
                Sigma = K_ss - (beta')*beta;
                
                % Get Acquisition values
                acq = mu + 0.5*diag(Sigma);
                
                % Choose highest value
                [~,idx] = max(acq);
                
                % Get next function value
                X = [X, X_(:,idx)];
                controlUnit.ClassPoseGraphOptimization.Gamma1 = X(1,i+1);
                controlUnit.ClassPoseGraphOptimization.Gamma2 = X(2,i+1);        
                [controlUnit,~] = controlUnit.mapping(estPath);
                results = controlUnit.compare(4);
                f = [f; -results.error];
            end
            
            [~,idx] = max(f);
            results.bestParam = X(:,idx);
            results.X = X;
            results.f = f;
        end
    end
    
    methods (Static)
        function [K] = se_kernel(X1,X2)
            % Squared-exponential covariance function (Kernel) with equal characteristic length
            % in every dimension, it is a measure of similarity
            %
            % Formular:
            %   k(x,x') = sigma_f^2 * exp(-(1/(2*(l^2))) * (x - x')^2) + sigma_n^2 kroneckerDelta_qp
            %
            % Syntax:
            %   [K] = se_kernel(X1,X2)
            %
            % Description:
            %   Generates a Covariance Matrix between the points given in X1 and X2.
            %   This covariance matrix can be used for instance for Gaussian Processes
            %
            % Input: 
            %   X1, X2: Matrices of data points with dimension D x n, D x m
            %
            % Output:
            %   K - covariance matrix (n x m)
            %
            % Date: 17. August, 2016
            % Author: Nils Rottman

            % Rewriting parameters in variables
            sigma_n = 1;
            l = 1;

            % computing the dimension of the kernel matrix
            l1 = length(X1(1,:));
            l2 = length(X2(1,:));

            % providing the kernel matrix for effcient computing
            K = zeros(l1,l2);

            % Computing the kernel for each element of the kernel matrix
            for i=1:1:l1
                for j=1:1:l2
                    K(i,j) = sigma_n^2 * exp(-(1/(2*(l^2))) * (norm(X1(:,i)-X2(:,j)))^2);
                end
            end
        end
    end
end