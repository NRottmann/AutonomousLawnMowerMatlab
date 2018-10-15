classdef SymbioticParticleFilter
    % Symbiotic Particle Filter class for autonomous lawn mower
    % Methods:
    %   ParticleFilter(numParticles,polyMap,p0)
    %   obj = update(obj,odometryData,sensorData)
    %   posParticles = getParticlePosition(obj)
    %   posParticles = getParticlePositionOrientationWeight(obj)
    %   rating = getParticleRating(obj)
    %   [mu,sigma] = getMeanVariance(obj)
    
    % Nils Rottmann (Nils.Rottmann@rob.uni-luebeck.de)
    % 15.10.2018
    
    properties
        NumParticles;                   % Number of particles used
        Particles;                      % Array of particles with every
                                        % row is one particle and contains
                                        % [x; y; phi; weight]
        DP;
        X;
        
        PolyMap;                        % The map, represented by polygons

        GrassSensor;                    % Instance of the class GrassSensor
        OdometryModel;                  % Instance of the class OdometryModel
        WallFollower;                   % Instance of the class WallFollower
        RandomController;               % Instance of the class RandomController
        
        GlobalLocalization;             % boolean, which tells us if global loclaization is required
        
        Counter;
    end
    
    methods
        function obj = SymbioticParticleFilter(numParticles,polyMap,p0,...
                            grassSensor,odometryModel,wallFollower,randomController)
            % This is the constructor of the class
            % Syntax:
            %       obj = ParticleFilter(nParticles,polyMap,p0)
            % Input:
            %   numParticles:   The number of particles used
            %   polyMap:     	The map as polygon
            %   p0:             A start Pose for the particles with 
            %                   p = [x,y,phi,known]^T, if known = 0, we
            %                   start with global localization
            % Output:
            %   obj:            Intance of the class ParticleFilter
            
            % Check for correct dimensions
            % TODO: do dimensions check
            
            % Allocate Parameters
            obj.NumParticles = numParticles;
            obj.PolyMap = polyMap;
            obj.GrassSensor = grassSensor;
            obj.OdometryModel = odometryModel;
            obj.WallFollower = wallFollower;
            obj.RandomController = randomController;
            
            % Initialize variables
            obj.DP = zeros(2,1);
            obj.X = zeros(2,2);
            
            % Initialize Particles
            obj.Particles = zeros(4, numParticles);
            if p0(4) == 1
                for i = 1:1:numParticles
                    obj.Particles(:,i) = [p0(1); p0(2); ...
                                        p0(3); 1/obj.NumParticles];
                end
                obj.GlobalLocalization = false;
            else
                obj.GlobalLocalization = true;
            end
        end
        
        function [obj,u] = update(obj,sensorData,motionData,p0)
            % This is the update function of the class which holds the
            % symbiotic particle filter algorithm
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
            
            % TODO: Use noise corrupted data for gloab localization
            
            %% Decide wether we are in global loclaization mode or not
            % TODO: add parameters to config
            L_min = 0.5;
            e_max = 10^(-5);
            comparisonResult.foundPosition = false;
       
            x_new = p0(1:2);
            if obj.GlobalLocalization
                % Generate control signals
                [obj.WallFollower,u] = obj.WallFollower.wallFollowing(sensorData);
                if obj.WallFollower.Mode == 0       % no wall following yet
                    obj.DP = x_new;
                    obj.X = [x_new, x_new];
                else
                    L = norm(x_new - obj.X(:,1));
                    e = SymbioticParticleFilter.lineSegmentationError(obj.X(:,1),obj.X(:,2),x_new);
                    if (L < L_min) || (e < e_max)
                        obj.X(:,2) = x_new;
                    else
                        obj.DP = [obj.DP obj.X(:,2)];
                        obj.X = [obj.X(:,2) x_new];
                        comparisonResult = SymbioticParticleFilter.compareHistory(obj.DP,obj.PolyMap);
                    end
                    if comparisonResult.foundPosition
                        obj.GlobalLocalization = false;
                        % Initialize Particles, TODO: Optimize this, maybe
                        % use Machine Learning, Reinforcement Learning
                        for i = 1:1:obj.NumParticles
                            x = normrnd(comparisonResult.position(1),0.2);
                            y = normrnd(comparisonResult.position(2),0.2);
                            phi = rand()*2*pi;
                            obj.Particles(:,i) = [x; y; phi; 1/obj.NumParticles];
                        end
                    end
                end
            else
                % Generate Control signals
                obj.Counter = obj.Counter + 1;
                if obj.Counter > 1000
                    odometryData.deltaR1 = obj.OdometryModel.DeltaR1;
                    odometryData.deltaR2 = obj.OdometryModel.DeltaR2;
                    [obj.RandomController,u] = obj.RandomController.randomControl(sensorData,odometryData);
                else
                    [obj.WallFollower,u] = obj.WallFollower.wallFollowing(sensorData);
                end
                %% Update
                % For all Particles, update them according to odometryData and
                % allocate weights according to measurement data
                for i = 1:1:obj.NumParticles
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
                N_eff = 1 / sum(obj.Particles(4,:).^2)

                %% Resampling
                % Do only resampling if N_eff < a*N
                out = get_config('particleFilter');
                treshResample =  out.treshResample;
                if (N_eff < treshResample * obj.NumParticles)
                    % Create Weighting vector
                    weightVec = zeros(obj.NumParticles+1,1);
                    for i=1:1:obj.NumParticles
                        weightVec(i+1) = weightVec(i) + obj.Particles(4,i);
                    end
                    % Systematic resampling
                    X = (0:1/obj.NumParticles:(obj.NumParticles-1)/obj.NumParticles) + rand();
                    X(X > 1) = X(X > 1) - 1;
                    [numSamples,~] = histcounts(X,weightVec);
                    tempParticles = zeros(4,obj.NumParticles);
                    idx = 1;
                    for i=1:1:obj.NumParticles
                        for j=1:1:numSamples(i)
                            tempParticles(:,idx) = obj.Particles(:,i);
                            idx = idx + 1;
                        end
                    end
                    % Allocate Particles and set new NumParticles
                    obj.Particles = tempParticles;
                end 
            end
        end
             
       % TODO: Check which functions are really required
        function posParticles = getParticlePosition(obj)
            % Function to give back the positions of the particles
            posParticles = obj.Particles(1:2,1:obj.NumParticles);
        end
        
        function posParticles = getParticlePositionOrientationWeight(obj)
            % Function to get the positions of the particles with the 
            % weight and the orientation 
            posParticles = obj.Particles(:,1:obj.NumParticles);
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
            mu = mean(obj.Particles(1:3,1:obj.NumParticles),2)';
            sigma = var(obj.Particles(1:3,1:obj.NumParticles),2)';
        end
    end
    
    methods (Static)
    	function e = lineSegmentationError(x0,x1,xTest)
            % TODO: Add description
            %
            % input: 
            % output:

            v = x1 - x0;
            phi = atan2(v(2),v(1));
            
            vTest = xTest - x0;
            phiTest = atan2(vTest(2),vTest(1));
            psi = phiTest - phi;
            
            e = (sin(psi)*norm(vTest))^2;
        end
        
        function result = compareHistory(DP,polyMap)
            % TODO: Add description
            
            % TODO: Change cmin to config script
            c_min = 0.3;
            
            [phiCum_DP, LCum_DP] = getCumOrientation(DP);
            DP_map = [polyMap.x polyMap.x(2:end); polyMap.y polyMap.y(2:end)];
            m = length(polyMap.x) - 1;
            valid = zeros(m,1);
            for j=1:1:m
                [phiCum_map, LCum_map] = getCumOrientation(DP_map(:,1:(m+j)));
                corr = getCorrelation(phiCum_DP,LCum_DP,phiCum_map,LCum_map);
                if corr < c_min
                    valid(j) = 1;
                end
            end
            sum(valid)
            if sum(valid) == 1
                result.foundPosition = true;
                idx = find(valid);
                result.position = [polyMap.x(idx); polyMap.y(idx)];
            else
                result.foundPosition = false;
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
                % TODO: add description
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

