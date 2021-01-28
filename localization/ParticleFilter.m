classdef ParticleFilter
    % Particle Filter class for autonomous lawn mower
    %
    % Methods:
    %   ParticleFilter
    %
    % Nils Rottmann (Nils.Rottmann@rob.uni-luebeck.de)
    % 21.01.2021
    
    properties
        % Storage Variables
        Particles;                      % Array of particles with every
                                        % row is one particle and contains
                                        % [x; y; phi; weight]
        Map;                            % A map of the environment
        CounterMeasurements;            % A counter for the measurements taken
        ParticlesMeasurements;          % Storage array for the measurements of the particles
        Measurements;                   % Storage array for the true measurements received by the real sensor 
        
        % Parameter
        N_P;                            % Number of particles used
        PoseVariance;                   % Variances for distributing the particles around an initial pose estimate
        N_M;                            % Number of measurements before updating the particles
        IncreaseNoise;                  % Factor which increases the noise of the odometry model for the particles
        ThresholdResampling;            % Threshold for resampling
        
        % Classes
        OdometryModel;                  % This model is used for simulating particle movements based on the odometry data
        GrassSensor;                    % This model is used to simulate the sensor measurements for the particles  
    end
    
    methods
        function obj = ParticleFilter(map)
            % This is the constructor of the class
            % Syntax:
            %       obj = ParticleFilter(map)
            % Input:
            %   map:    A map of the environment
          
            % Allocate Parameters
            obj.Map = map;
            
            % Initialize classes
            obj.GrassSensor = GrassSensor(map);
            obj.OdometryModel = OdometryModel;

            % Parameters
            out = get_config('particleFilter');
            obj.N_P = out.n_P;
            obj.PoseVariance = out.poseVariance;
            obj.N_M = out.n_M;
            obj.IncreaseNoise = out.increaseNoise;
            obj.ThresholdResampling = out.thresholdResampling;
            
            % Initialize Measurement Counter
            obj.CounterMeasurements = 0;
            
            % Initialize Arrays for the particles
            obj.Particles = zeros(4, obj.N_P);
            obj.ParticlesMeasurements{1} = zeros(obj.N_M,obj.N_P);
            obj.Measurements{1} = zeros(obj.N_M,1);
            obj.ParticlesMeasurements{2} = zeros(obj.N_M,obj.N_P);
            obj.Measurements{2} = zeros(obj.N_M,1);   
        end
        
        function [obj] = initializeParticles(obj,pose,mode)
            % This method initializes the particles for the particle filter
            % dependend on the mode chosen
            % Syntax:
            %       [obj] = initializeParticles(obj,pose,mode)
            % Input:
            %   pose:           Pose around which the particles should be
            %                   spread
            %   mode:           Mode how the particles should be spread
            %       1:          Around the given pose within the config
            %                   defined variance
            %       2:          Randomly over the whole field
            %       3:          Exactly at the given pose
            
            if mode == 1
                % Spread particles around the pose estimate
                for i = 1:1:obj.N_P
                    % TODO: Spread Particles more around the current
                    % direction
                    obj.Particles(1,i) = normrnd(pose(1),obj.PoseVariance(1));
                    obj.Particles(2,i) = normrnd(pose(2),obj.PoseVariance(2));
                    obj.Particles(3,i) = normrnd(pose(3),obj.PoseVariance(3));
                    obj.Particles(4,i) = 1/obj.N_P;
                end
            elseif mode == 2
                % Spread particles randomly over the whole field
                dx = obj.Map.XWorldLimits(2) - obj.Map.XWorldLimits(1);
                dy = obj.Map.YWorldLimits(2) - obj.Map.YWorldLimits(1);
                for i = 1:1:obj.N_P
                    obj.Particles(:,i) = [rand()*dx + obj.Map.XWorldLimits(1); ...
                                          rand()*dy + obj.Map.YWorldLimits(1); ...
                                          rand()*2*pi; 1/obj.N_P];
                end
            elseif mode == 3
                % Set all particles to the exact same initial state
                for i = 1:1:obj.N_P
                    obj.Particles(:,i) = [pose(1); pose(2); ...
                                            pose(3); 1/obj.N_P];
                end
            else
                error('Wrong mode chosen!')
            end

            % Initialize measurement arrays for saving measurement history,
            for i=1:1:2
                obj.ParticlesMeasurements{i} = zeros(obj.N_M,obj.N_P);
                obj.Measurements{i} = zeros(obj.N_M,1);
            end
        end
        
        function obj = updateParticles(obj,sensorData,odometryData)
            % This methods updates the particles from the particle filter
            % based on the measurements (odometry, sensorData)P
            % Syntax:
            %       obj = update(obj,sensorData,odometryData)
            % Input:
            %   obj:            Object of the Particle Filter class
            %   odometryData:   deltaR1, deltaT, deltaR2
            %   sensorData:     data from the sensor estimation
            % Output:
            %   obj:            Instance of the class ParticleFilter
            
            % Load odometry measurements into the odometry model
            obj.OdometryModel.DeltaR1 = odometryData.DeltaR1;
            obj.OdometryModel.DeltaT = odometryData.DeltaT;
            obj.OdometryModel.DeltaR2 = odometryData.DeltaR2;
            
            % For all Particles, update them according to odometryData and
            % allocate weights according to measurement data, thereby we
            % update only every N_S measurements
            obj.CounterMeasurements = obj.CounterMeasurements + 1;
            for i = 1:1:obj.N_P
                % Move Particle, add noise
                obj.Particles(1:3,i) = obj.OdometryModel.odometryPose(obj.Particles(1:3,i),true,obj.IncreaseNoise); 
                % Simulate measurements based on the pose of the particles
                sensorParticleData = obj.GrassSensor.measure(obj.Particles(1:3,i));
                % Store measurements
                obj.ParticlesMeasurements{1}(obj.CounterMeasurements,i) = sensorParticleData.right;
                obj.ParticlesMeasurements{2}(obj.CounterMeasurements,i) = sensorParticleData.left;
            end

            % Allocate real measurements to storage array
            obj.Measurements{1}(obj.CounterMeasurements) = sensorData.right;
            obj.Measurements{2}(obj.CounterMeasurements) = sensorData.left;
            
            % If enough measurements have been taken, update weights for all particles
            if (obj.CounterMeasurements == obj.N_M)
                obj.CounterMeasurements = 0;
                for i = 1:1:obj.N_P
                    % TODO: Find intelligent update strategies
%                     w1 = 1 - abs(mean(obj.Measurements{1}(:)) ...
%                                 - mean(obj.ParticlesMeasurements{1}(:,i)));
%                     w2 = 1 - abs(mean(obj.Measurements{2}(:)) ...
%                                 - mean(obj.ParticlesMeasurements{2}(:,i)));
%                     obj.Particles(4,i) = ((w1 + w2)/2) + 0.1;   
                    if (obj.Measurements{1} == obj.ParticlesMeasurements{1}(1,i) && ...
                            obj.Measurements{2} == obj.ParticlesMeasurements{2}(1,i))
                        obj.Particles(4,i) = 0.9;
                    else
                        obj.Particles(4,i) = 0.1;
                    end
                end
            end

            % Normalize weights and calculate effective number of particles
            obj.Particles(4,:) = obj.Particles(4,:) / sum(obj.Particles(4,:));
            N_eff_rel = (1 / sum(obj.Particles(4,:).^2)) / obj.N_P; 

            % Resampling, do only if N_eff/N < threshold
            if (N_eff_rel < obj.ThresholdResampling)
                disp('resampling')
                % Create Weighting vector
                weightVec = zeros(obj.N_P+1,1);
                for i=1:1:obj.N_P
                    weightVec(i+1) = weightVec(i) + obj.Particles(4,i);
                end
                % Systematic resampling
                X = (0:1/obj.N_P:(obj.N_P-1)/obj.N_P) + rand();
                X(X > 1) = X(X > 1) - 1;
                [numSamples,~] = histcounts(X,weightVec);
                tempParticles = zeros(4,obj.N_P);
                idx = 1;
                for i=1:1:obj.N_P
                    for j=1:1:numSamples(i)
                        tempParticles(:,idx) = obj.Particles(:,i);
                        idx = idx + 1;
                    end
                end
                % Allocate new Particles
                obj.Particles = tempParticles;
            end 
        end
  
        function [pose,sigma] = getPoseEstimate(obj)
            % This methods gives back a pose estimate
            % Syntax:
            %       [pose,variance] = getPoseEstimate(obj)
            % Input:
            %
            % Output:
            %   pose:           The pose estimate
            %   variance:       The variance for the given pose estimate
            pose = mean(obj.Particles(1:3,1:obj.N_P),2)';
            sigma = std(obj.Particles(1:3,1:obj.N_P),0,2)';
        end
    end
end

