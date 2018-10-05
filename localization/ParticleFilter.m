classdef ParticleFilter
    % Particle Filter class for autonomous lawn mower
    % Methods:
    %   ParticleFilter(numParticles,polyMap,p0)
    %   obj = update(obj,odometryData,sensorData)
    %   posParticles = getParticlePosition(obj)
    %   posParticles = getParticlePositionOrientationWeight(obj)
    %   rating = getParticleRating(obj)
    %   [mu,sigma] = getMeanVariance(obj)
    
    % Nils Rottmann (Nils.Rottmann@rob.uni-luebeck.de)
    % 01.10.2018
    
    properties
        NumParticles;                   % Array of min and max Particles
        ActParticles;                   % Actual number of particles used
        Particles;                      % Array of particles with every
                                        % row is one particle and contains
                                        % [x; y; phi; weight]
        PolyMap;

        GrassSensor;                    % Instance of the class GrassSensor
        OdometryModel;                  % Instance of the class OdometryModel
    end
    
    methods
        function obj = ParticleFilter(numParticles,polyMap,p0,grassSensor,odometryModel)
            % This is the constructor of the class
            % Syntax:
            %       obj = ParticleFilter(nParticles,polyMap,p0)
            % Input:
            %   numParticles:   The number of particles as [min; max]
            %   polyMap:     	The map as polygon
            %   p0:             A start Pose for the particles with 
            %                   p = [x,y,phi,known]^T, if known = 0, the
            %                   starting poses will be chosen randomly
            % Output:
            %   obj:            Intance of the class ParticleFilter
            
            % Check for correct dimensions
            if (size(numParticles) ~= [2 1])
                error('Size of input nParticles is not correct!')
            end
            
            % Allocate Parameters
            obj.NumParticles = numParticles;
            obj.PolyMap = polyMap;
            obj.GrassSensor = grassSensor;
            obj.OdometryModel = odometryModel;
            
            % Initialize Particles
            obj.Particles = zeros(4, numParticles(2));
            if p0(4) == 1
                obj.ActParticles = obj.NumParticles(1);
                for i = 1:1:obj.ActParticles
                    obj.Particles(:,i) = [p0(1); p0(2); ...
                                        p0(3); 1/obj.ActParticles];
                end
            else
                obj.ActParticles = obj.NumParticles(2);
                dx = polyMap.XWorldLimits(2) - polyMap.XWorldLimits(1);
                dy = polyMap.YWorldLimits(2) - polyMap.YWorldLimits(1);
                for i = 1:1:obj.ActParticles
                    obj.Particles(:,i) = [rand()*dx + polyMap.XWorldLimits(1); ...
                                          rand()*dy + polyMap.YWorldLimits(1); ...
                                          rand()*2*pi; 1/obj.ActParticles];
                end   
            end
        end
        
        function obj = update(obj,sensorData,motionData,p0)
            % This is the update function of the class which holds the
            % particle filter algorithm
            % Syntax:
            %       obj = update(obj,odometryData,sensorData)
            % Input:
            %   obj:            Object of the Particle Filter class
            %   odometryData:   deltaR1, deltaT, deltaR2
            %   sensorData:     data from the sensor estimation
            % Output:
            %   obj:            Instance of the class ParticleFilter
            
            %% Calculate odometry data
            obj.OdometryModel = obj.OdometryModel.odometryData(p0, motionData);
            
            %% Update
            % For all Particles, update them according to odometryData and
            % allocate weights according to measurement data
            tic;
            for i = 1:1:obj.ActParticles
                % Move Particle, add noise
                obj.Particles(1:3,i) = obj.OdometryModel.odometryPose(obj.Particles(1:3,i),true);
                                        
                % Allocate Weights, therefore check what the sensor should
                % measure depending on the particles position and compare
                % this with the actual measurement
                out = get_config('particleFilter');
                weightFactors =  out.weightFactors;
                [sensorParticleData] = obj.GrassSensor.measure(obj.Particles(1:3,i));
                if ((sensorParticleData.right == sensorData.right) && (sensorParticleData.left == sensorData.left))
                    obj.Particles(4,i) = weightFactors(1);
                elseif ((sensorParticleData.right == sensorData.right) || (sensorParticleData.left == sensorData.left))
                    obj.Particles(4,i) = weightFactors(2);
                else
                    obj.Particles(4,i) = weightFactors(3);    
                end 
            end
            
            % Normalize weights and calculate effective number of particles
            obj.Particles(4,:) = obj.Particles(4,:) / sum(obj.Particles(4,:));
            N_eff = 1 / sum(obj.Particles(4,:).^2);
            disp('Update step: ')
            toc
            
            %% Resampling
            % Do only resampling if N_eff < a*N
            out = get_config('particleFilter');
            treshResample =  out.treshResample;
            if (N_eff < treshResample * obj.ActParticles)
                
                % Check how good the Particle Filter estimates the position 
                % and change the particle Size according to this
                % TODO: Check if this strategy is valid
                rating = obj.getParticleRating();
                scale = obj.NumParticles(2) - obj.NumParticles(1);
                scale = scale * (1-rating);
                if (1-rating) < 10^(-2) % handle division by 0
                    scale = obj.NumParticles(1);
                else
                    scale = obj.NumParticles(1) + ceil(scale);
                end
                newNumParticles = scale;
                
                % Create Weighting vector
                weightVec = zeros(obj.ActParticles+1,1);
                for i=1:1:obj.ActParticles
                    weightVec(i+1) = weightVec(i) + obj.Particles(4,i);
                end
                % Systematic resampling
                tic;
                X = (0:1/newNumParticles:(newNumParticles-1)/newNumParticles) + rand();
                X(X > 1) = X(X > 1) - 1;
                [numSamples,~] = histcounts(X,weightVec);
                tempParticles = zeros(4,obj.NumParticles(2));
                idx = 1;
                for i=1:1:newNumParticles
                    for j=1:1:numSamples(i)
                        tempParticles(:,idx) = obj.Particles(:,i);
                        idx = idx + 1;
                    end
                end
                disp('Systematic Resampling: ');
                toc
%                 % Random resampling
%                 tempParticles = zeros(4,obj.NumParticles(2));
%                 for i=1:1:newNumParticles
%                     randNum = rand();
%                     k = 1;
%                     while (randNum > weightVec(k+1))
%                         k = k + 1;
%                     end
%                     tempParticles(:,i) = obj.Particles(:,k);
%                 end
                % Allocate Particles and set new NumParticles
                obj.Particles = tempParticles;
                obj.ActParticles = newNumParticles;
            end            
        end
             
       % TODO: Check which functions are really required
        function posParticles = getParticlePosition(obj)
            % Function to give back the positions of the particles
            posParticles = obj.Particles(1:2,1:obj.ActParticles);
        end
        
        function posParticles = getParticlePositionOrientationWeight(obj)
            % Function to get the positions of the particles with the 
            % weight and the orientation 
            posParticles = obj.Particles(:,1:obj.ActParticles);
        end
     
        % TODO: Is this really useful or exists their already an idea?
        function rating = getParticleRating(obj)
            % Compute the likelihood of the position
            
            % Get the positions and the max and mins
            positions = obj.getParticlePositionOrientationWeight();
            [~,minX] = min(positions(1,:));
            minX = abs(positions(1,minX));
            [~,maxX] = max(positions(1,:));
            maxX = abs(positions(1,maxX));
            [~,minY] = min(positions(2,:));
            minY = abs(positions(2,minY));
            [~,maxY] = max(positions(2,:));
            maxY = abs(positions(2,maxY));
            
            % Calculate the square of the pf
            bottom = maxX - minX;
            left = maxY - minY;
            sizeOf = bottom*left;
            maxMapSize = obj.PolyMap.XWorldLimits(2) * obj.PolyMap.YWorldLimits(2);
            rating = 1 - (sizeOf/maxMapSize);
            if rating > 1
                rating = 1;
            end
        end
        
        function [mu,sigma] = getMeanVariance(obj)
            mu = mean(obj.Particles(1:3,1:obj.ActParticles),2)';
            sigma = var(obj.Particles(1:3,1:obj.ActParticles),2)';
        end
    end
end

