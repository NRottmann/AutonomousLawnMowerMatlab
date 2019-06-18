classdef ParticleFilter
    % Symbiotic Particle Filter class for autonomous lawn mower
    %
    % Methods:
    %   ParticleFilter
    %
    % Nils Rottmann (Nils.Rottmann@rob.uni-luebeck.de)
    % 06.03.2019
    
    properties
        % Storage Variables
        Particles;                      % Array of particles with every
                                        % row is one particle and contains
                                        % [x; y; phi; weight]
        PolyMap;                        % A map of the environment
        CounterMeasurements;            % A counter for the measurements taken
        ParticlesMeasurements;          % Storage array for the measurements of the particles
        Measurements;                   % Storage array for the true measurements received by the real sensor 
                                                                                
        % Parameter
        N_P;                            % Number of particles used
        PoseVariance;                   % Variances for distributing the particles around an initial pose estimate
        N_M;                            % Number of measurements before updating the particles
        IncreaseNoise;                  % Factor which increases the noise of the odometry model for the particles
        N_S;                            % Number sensors used, (1 or 2)
        ThresholdResampling;            % Threshold for resampling
        
        Parallel;                       % Boolean of parallel computation required
        A;                              % Odometrie model parameters
        PosRight;
        PosLeft;
        
        % Classes
        OdometryModel;                  % This model is used for simulating particle movements based on the odometry data
        GrassSensor;                    % This model is used to simulate the sensor measurements for the particles  
    end
    
    methods
        function obj = ParticleFilter(polyMap)
            % This is the constructor of the class
            % Syntax:
            %       obj = ParticleFilter(nParticles,polyMap,p0)
            % Input:
            %   polyMap:     	The map as polygon
            % Output:
          
            % Allocate Parameters
            obj.PolyMap = polyMap;
            
            % Initialize classes
            obj.GrassSensor = GrassSensor(polyMap);
            obj.OdometryModel = OdometryModel;

            % Parameters
            out = get_config('particleFilter');
            obj.N_P = out.n_P;
            obj.PoseVariance = out.poseVariance;
            obj.N_M = out.n_M;
            obj.IncreaseNoise = out.increaseNoise;
            obj.N_S = out.n_S;
            
            out = get_config('computation');
            obj.Parallel = out.parallel; 
            
            out = get_config('odometryModelNoise');
            obj.A = out.a; 
            
            out = get_config('Sensor');
            obj.PosRight =  out.posRight;
            obj.PosLeft = out.posLeft;
            
            % Initialize Measurement Counter
            obj.CounterMeasurements = 0;
            
            % Initialize Arrays
            obj.Particles = zeros(4, obj.N_P);
            if obj.N_S == 1
                obj.ParticlesMeasurements{1} = zeros(obj.N_M,obj.N_P);
                obj.Measurements{1} = zeros(obj.N_M,1);
            elseif obj.N_S == 2
                obj.ParticlesMeasurements{1} = zeros(obj.N_M,obj.N_P);
                obj.Measurements{1} = zeros(obj.N_M,1);
                obj.ParticlesMeasurements{2} = zeros(obj.N_M,obj.N_P);
                obj.Measurements{2} = zeros(obj.N_M,1);   
            else
                error('Wrong number N_S for sensors used!')
            end
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
            
            % Initialize Particle Arrays
            obj.Particles = zeros(4, obj.N_P);
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
                dx = obj.PolyMap.XWorldLimits(2) - obj.PolyMap.XWorldLimits(1);
                dy = obj.PolyMap.YWorldLimits(2) - obj.PolyMap.YWorldLimits(1);
                for i = 1:1:obj.N_P
                    obj.Particles(:,i) = [rand()*dx + obj.PolyMap.XWorldLimits(1); ...
                                          rand()*dy + obj.PolyMap.YWorldLimits(1); ...
                                          rand()*2*pi; 1/obj.N_P];
                end
            elseif mode == 3
                for i = 1:1:obj.N_P
                    obj.Particles(:,i) = [pose(1); pose(2); ...
                                            pose(3); 1/obj.N_P];
                end
            else
                error('Wrong mode chosen!')
            end

            % Initialize measurement arrays for saving measurement history,
            for i=1:1:obj.N_S
                obj.ParticlesMeasurements{i} = zeros(obj.N_M,obj.N_P);
                obj.Measurements{i} = zeros(obj.N_M,1);
            end
        end
        
        function [obj] = updateParticles(obj,sensorData,odometryData)
            % This methods updates the particles from the particle filter
            % based on the measurements (odometry, sensorData)P
            % Syntax:
            %       obj = update(obj,odometryData,sensorData)
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
            % update only every N
            obj.CounterMeasurements = obj.CounterMeasurements + 1;
            
            % Parallized computing
            if obj.Parallel
                particles = obj.Particles(1:3,:);
                measurements = zeros(obj.N_S,obj.N_P);
                a = obj.A;
                DeltaR1 = obj.OdometryModel.DeltaR1;
                DeltaT = obj.OdometryModel.DeltaT;
                DeltaR2 = obj.OdometryModel.DeltaR2;
                posRight = obj.PosRight;
                posLeft = obj.PosLeft;
                x_vertices = obj.PolyMap.x;
                y_vertices = obj.PolyMap.y;
                n_s = obj.N_S;
                parfor i = 1:1:obj.N_P
                    % Move Particle, add noise
                    deltaR1 = DeltaR1 - sampleNormalDistribution(a(1)*abs(DeltaR1) + a(2)*DeltaT);
                    deltaT = DeltaT - sampleNormalDistribution(a(3)*DeltaT + a(4)*abs(DeltaR1+DeltaR2));
                    deltaR2 = DeltaR2 - sampleNormalDistribution(a(1)*abs(DeltaR2) + a(2)*DeltaT);
                    p0 = particles(:,i);
                    p0 = p0 + [cos(p0(3)+deltaR1), 0; sin(p0(3)+deltaR1), 0; 0, 1] ...
                            * [deltaT; (deltaR1+deltaR2)];
                    % Simulate measurements based on the pose of the particles
                    % Orientation Matrix
                    R = [cos(p0(3)) -sin(p0(3)); sin(p0(3)) cos(p0(3))];
                    % Caluclate the actual positions of the sensors
                    pR = p0(1:2) + R*posRight;
                    pL = p0(1:2) + R*posLeft;
                    % Make the measurements
                    right = inpolygon(pR(1),pR(2),x_vertices,y_vertices);
                    left = inpolygon(pL(1),pL(2),x_vertices,y_vertices);

                    % Store pose and weights
                    particles(:,i) = p0;

                    % Store measurements
                    if n_s == 1
                        measurements(:,i) = right;
                    elseif n_s == 2
                        measurements(:,i) = [right; left];
                    else
                        error('Wrong number of sensors chosen (n_S)');
                    end           
                end
                % Allcoate
                obj.Particles(1:3,:) = particles;
                if obj.N_S == 1
                    obj.ParticlesMeasurements{1}(obj.CounterMeasurements,:) = measurements(1,:);
                elseif obj.N_S == 2
                    obj.ParticlesMeasurements{1}(obj.CounterMeasurements,:) = measurements(1,:);
                    obj.ParticlesMeasurements{2}(obj.CounterMeasurements,:) = measurements(2,:);
                else
                    error('Wrong number of sensors chosen (n_S)');
                end   
            else
                for i = 1:1:obj.N_P
                    % Move Particle, add noise
                    obj.Particles(1:3,i) = obj.OdometryModel.odometryPose(obj.Particles(1:3,i),true,obj.IncreaseNoise);
                    % Simulate measurements based on the pose of the particles
                    sensorParticleData = obj.GrassSensor.measure(obj.Particles(1:3,i));
                    % Store measurements
                    if obj.N_S == 1
                        obj.ParticlesMeasurements{1}(obj.CounterMeasurements,i) = sensorParticleData.right;
                    elseif obj.N_S == 2
                        obj.ParticlesMeasurements{1}(obj.CounterMeasurements,i) = sensorParticleData.right;
                        obj.ParticlesMeasurements{2}(obj.CounterMeasurements,i) = sensorParticleData.left;
                    else
                        error('Wrong number of sensors chosen (n_S)');
                    end           
                end
            end

            % Allocate real measurements to storage array
            if obj.N_S == 1
                obj.Measurements{1}(obj.CounterMeasurements) = sensorData.right;
            elseif obj.N_S == 2
                obj.Measurements{1}(obj.CounterMeasurements) = sensorData.right;
                obj.Measurements{2}(obj.CounterMeasurements) = sensorData.left;
            end
            % If enough measurements have been taken, update weights for all particles
            if (obj.CounterMeasurements == obj.N_M)
                obj.CounterMeasurements = 0;
                % TODO: Find intelligent update strategies
                for i=1:1:obj.N_P
                    if obj.N_S == 1
                        obj.Particles(4,i) = 1 - abs(mean(obj.Measurements{1}(:)) ...
                                    - mean(obj.ParticlesMeasurements{1}(:,i)));
                    elseif mode == 2
                        w1 = 1 - abs(mean(obj.Measurements{1}(:)) ...
                                    - mean(obj.ParticlesMeasurements{1}(:,i)));
                        w2 = 1 - abs(mean(obj.Measurements{2}(:)) ...
                                    - mean(obj.ParticlesMeasurements{2}(:,i)));
                        obj.Particles(4,i) = w1 + w2;
                    end
                end
            end

            % Normalize weights and calculate effective number of particles
            obj.Particles(4,:) = obj.Particles(4,:) / sum(obj.Particles(4,:));
            N_eff = 1 / sum(obj.Particles(4,:).^2);

            % Resampling, do only if N_eff < a*N
            if (N_eff < obj.ThresholdResampling * obj.N_P)
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
  
        function [pose,variance] = getPoseEstimate(obj)
            % This methods gives back a pose estimate
            % Syntax:
            %       [pose,variance] = getPoseEstimate(obj)
            % Input:
            %
            % Output:
            %   pose:           The pose estimate
            %   variance:       The variance for the given pose estimate
            pose = mean(obj.Particles(1:3,1:obj.N_P),2)';
            variance = std(obj.Particles(1:3,1:obj.N_P),0,2)';
        end
    end
end

