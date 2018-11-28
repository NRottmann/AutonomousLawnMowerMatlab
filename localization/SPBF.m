classdef SPBF
    % TODO: This symbiotic particle Filter algorithm has still to be reviewed and correct 
    %
    % Sigma-Point Bayesian Filter class for autonomous lawn mower
    % Methods:
    %   ...
    
    % Nils Rottmann (Nils.Rottmann@rob.uni-luebeck.de)
    % 02.10.2018
    
    properties
        Nx;                     % 2nx + 1 sigma points are used
        PolyMap;                % Map as polygon structure
        Pose;                   % Pose estimate
        Covariance;             % Covariance estimate
        OdoNoise;               % Noise parameter of the odometry
        
        % Debugging
        SigmaPoints;
        Weights;
        CovTmp;
    end
    
    methods
        function obj = SPBF(nx,polyMap,p0)
            % This is the constructor of the class
            % Syntax:
            %       obj = SPBF(nx,polyMap,p0)
            % Input:
            %   nx:             2nx + 1 sigma points are used
            %   polyMap:     	The map as polygon
            %   p0:             A start Pose for the particles with 
            %                   p = [x,y,phi]^T
            % Output:
            %   obj:            Intance of the class ParticleFilter
            
            % Allocate Parameters
            obj.Nx = nx;        % TODO: Nx is dependend on the dimension of the problem
            obj.PolyMap = polyMap;
            
            % Initialize Pose and Covariance
            obj.Pose = p0;
            obj.Covariance = zeros(3);
            
            % Debugging
            obj.SigmaPoints = zeros(3,nx*2+1);
            obj.Weights = zeros(1,nx*2+1);
            obj.CovTmp = zeros(3);
            
            % get parameter from config data
            out = get_config('odometryModelNoise');
            obj.OdoNoise = out.a;
   
      
        end
        
        function obj = update(obj,odometryData,sensorData)
            % This is the update function of the class which holds the
            % algorithm
            % Syntax:
            %       obj = update(obj,odometryData,sensorData)
            % Input:
            %   obj:            Object of the Particle Filter class
            %   odometryData:   deltaR1, deltaT, deltaR2
            %   sensorData:     data from the sensor estimation
            % Output:
            %   obj:            Instance of the class ParticleFilter
            
            % Prediction step using a simple EKF
            x_hat = obj.Pose + ...
                        [odometryData.deltaT*cos(obj.Pose(3)+odometryData.deltaR1); ...
                            odometryData.deltaT*sin(obj.Pose(3)+odometryData.deltaR1); ...
                            odometryData.deltaR1+odometryData.deltaR2];
                        
            U = diag([obj.OdoNoise(1)*odometryData.deltaR1 + obj.OdoNoise(2)*odometryData.deltaT, ...
                        obj.OdoNoise(3)*odometryData.deltaT + obj.OdoNoise(4)*(odometryData.deltaR1+odometryData.deltaR2), ...
                        obj.OdoNoise(1)*odometryData.deltaR2 + obj.OdoNoise(2)*odometryData.deltaT]);
                    
            Fu = [-odometryData.deltaT*sin(obj.Pose(3)+odometryData.deltaR1), cos(obj.Pose(3)+odometryData.deltaR1), 0; ...
                    odometryData.deltaT*cos(obj.Pose(3)+odometryData.deltaR1), sin(obj.Pose(3)+odometryData.deltaR1), 0; ...
                    1, 0, 1];
                
            Fx = [1, 0, -odometryData.deltaT*sin(obj.Pose(3)+odometryData.deltaR1); ...
                    0, 1, odometryData.deltaT*cos(obj.Pose(3)+odometryData.deltaR1); ...
                    0, 0, 1];
            
            P_hat = Fx*obj.Covariance*Fx' + Fu*U*Fu';
            
            % Unscendet Transform
            sigmaPoints = zeros(3,2*obj.Nx+1);     	% Set of sigma points
            weights = zeros(1,2*obj.Nx+1);         	% Weights according to the sigma points
            
            sigmaPoints(:,end) = x_hat;
            weights(end) = 0.5;                         % TODO: find sufficient value for W0
            
            squareP = sqrtm(P_hat);
            W = (1-weights(end))/(2*obj.Nx);
            
            % TODO: Chechk if column or row is correct
            for i=1:1:obj.Nx
                tmp = sqrt(obj.Nx/(1-weights(end))) * squareP;
                sigmaPoints(:,i) = x_hat + tmp(:,i);
                sigmaPoints(:,i+obj.Nx) = x_hat - tmp(:,i);
                weights(i) = W;
                weights(i+obj.Nx) = W;
            end
            % TODO: Delete Debugging
            obj.SigmaPoints = sigmaPoints;
            
            % Update the weights using the measurements and normalize
            % TODO: is this weighting correct?
            for i=1:1:2*obj.Nx+1
                [sensorSigma] = grassSensor(sigmaPoints(:,i),obj.PolyMap);
                if ((sensorSigma.right == sensorData.right) && (sensorSigma.left == sensorData.left))
                    p = 0.75;
                elseif ((sensorSigma.right == sensorData.right) || (sensorSigma.left == sensorData.left))
                    p = 0.2;
                else
                    p = 0.05;
                end
                weights(i) = weights(i) * p;     
            end
            weights = weights/sum(weights);
            obj.Weights = weights;
            
            % Update Estimate and Covariance
            obj.Pose = sigmaPoints*weights';
            P_tmp = zeros(3);
            diff = sigmaPoints - x_hat;
            for i=1:1:2*obj.Nx+1
                P_tmp = P_tmp + weights(i) * diff(:,i) * diff(:,i)';
            end
            obj.Covariance = P_tmp;
        end
        
        function [x,P] = getPose(obj)
            % Function to give back the pose estimate and the variance
            x = obj.Pose;
            P = obj.Covariance;
        end
        
        function [sigmaPoints,weights] = getSigmas(obj)
            % Function to give back the pose estimate and the variance
            sigmaPoints = obj.SigmaPoints;
            weights = obj.Weights;
        end
    end
end

