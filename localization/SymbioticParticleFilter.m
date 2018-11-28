classdef SymbioticParticleFilter
    % TODO: This symbiotic particle Filter algorithm has still to be reviewed and correct 
    %
    % Symbiotic Particle Filter class for autonomous lawn mower
    % Methods:
    %   ParticleFilter(numParticles,polyMap,p0)
    %   obj = update(obj,odometryData,sensorData)
    %   posParticles = getParticlePosition(obj)
    %   posParticles = getParticlePositionOrientationWeight(obj)
    %   rating = getParticleRating(obj)
    %   [mu,sigma] = getMeanVariance(obj)
    %
    % Nils Rottmann (Nils.Rottmann@rob.uni-luebeck.de)
    % 05.11.2018
    
    properties
        NumParticles;                   % Number of particles used
        Particles;                      % Array of particles with every
                                        % row is one particle and contains
                                        % [x; y; phi; weight]
        DP;
        x_old;
        S;
        
        PoseMatch;
        
        PolyMap;                        % The map, represented by polygons
        
        PosRight;                       % Sensor position
        
        Corr;                           % Debug

        GrassSensor;                    % Instance of the class GrassSensor
        OdometryModel;                  % Instance of the class OdometryModel
        WallFollower;                   % Instance of the class WallFollower
        RandomController;       
        
        RandomControl;
        
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
            
            obj.RandomControl = false;
            
            % Initialize variables
            obj.DP = zeros(2,1);
            obj.x_old = zeros(2,1);
            obj.S = zeros(2,1);
            obj.PoseMatch = zeros(3,1);
            
            % Sensor position
            out = get_config('Sensor');
            obj.PosRight =  out.posRight;
            
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
        
        function [obj,u] = update(obj,sensorData,odometryData,p_corrupted)
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
            
            %% Set odometry data
            obj.OdometryModel.DeltaR1 = odometryData.DeltaR1;
            obj.OdometryModel.DeltaT = odometryData.DeltaT;
            obj.OdometryModel.DeltaR2 = odometryData.DeltaR2;
            
            %% Decide wether we are in global loclaization mode or not
            % TODO: add parameters to config
            L_min = 0.5;
            e_max = 10^(-2);
            U_min = 0.4;
            comparisonResult.foundPosition = false;
            
            % Transform to sensor position
            R = [cos(p_corrupted(3)) -sin(p_corrupted(3)); sin(p_corrupted(3)) cos(p_corrupted(3))];
            x_new = p_corrupted(1:2) + R*obj.PosRight;
            
            % Do the algorithm
            if obj.GlobalLocalization
                % Generate control signals
                [obj.WallFollower,u] = obj.WallFollower.wallFollowing(sensorData);
                if obj.WallFollower.Mode == 0       % no wall following yet
                    obj.DP = x_new;
                    obj.S = x_new;
                    obj.x_old = x_new;
                else
                    L = norm(x_new - obj.DP(:,end));
                    if (L < L_min)
                        obj.S = [obj.S, x_new];
                    else
                        S_tmp = [obj.S, x_new];
                        e = SymbioticParticleFilter.lineSegmentationError(S_tmp);
                        if (e < e_max)
                            obj.S = S_tmp;
                        else
                            obj.DP = [obj.DP, obj.x_old];
                            obj.S = [obj.x_old, x_new];                            
%                             comparisonResult = SymbioticParticleFilter.compareHistory(obj.DP,obj.PolyMap);
%                             obj.Corr = comparisonResult.corr;
                            % NEW
                            l_DP = 0;
                            for i=2:1:length(obj.DP(1,:))
                                l_DP = l_DP + norm(obj.DP(:,i) - obj.DP(:,i-1));
                            end
                            if l_DP > U_min*obj.PolyMap.Circumference
                                if l_DP > obj.PolyMap.Circumference % Delete last DP if too long
                                    obj.DP(:,1) = [];
                                end
                                comparisonResult = SymbioticParticleFilter.compareHistory(obj.DP,obj.PolyMap);
                                obj.Corr = comparisonResult.corr;
                            end
                        end
                    end
                    obj.x_old = x_new;
                    if comparisonResult.foundPosition
                        obj.GlobalLocalization = false;
                        % Initialize Particles, TODO: Optimize this, maybe
                        % use Machine Learning, Reinforcement Learning
                        % Transform from estimated sensor position to
                        % actual location of the robot
                        R = [cos(comparisonResult.phi) -sin(comparisonResult.phi); ...
                                    sin(comparisonResult.phi) cos(comparisonResult.phi)];
                        comparisonResult.position = comparisonResult.position - R*obj.PosRight;
                        for i = 1:1:obj.NumParticles
                            x = normrnd(comparisonResult.position(1),0.2);
                            y = normrnd(comparisonResult.position(2),0.2);
                            phi = normrnd(comparisonResult.phi,0.5);
                            obj.Particles(:,i) = [x; y; phi; 1/obj.NumParticles];
                        end
                        obj.PoseMatch = [comparisonResult.position; comparisonResult.phi];
                    end
                end
            else
                % Generate Control signals
                std_est = std(obj.Particles(1:3,1:obj.NumParticles),0,2)';
                % [obj.WallFollower,u] = obj.WallFollower.wallFollowing(sensorData);
                if std_est(3) > 0.1 && ~obj.RandomControl
                    [obj.WallFollower,u] = obj.WallFollower.wallFollowing(sensorData);
                else
                    obj.RandomControl = true;
                    [obj.RandomController,u] = obj.RandomController.randomControl(sensorData,odometryData);
                end
                %% Update
                % For all Particles, update them according to odometryData and
                % allocate weights according to measurement data
                for i = 1:1:obj.NumParticles
                    % Move Particle, add noise
                    incNoise = 2;
                    obj.Particles(1:3,i) = obj.OdometryModel.odometryPose(obj.Particles(1:3,i),true,incNoise);

                    % Allocate Weights, therefore check what the sensor should
                    % measure depending on the particles position and compare
                    % this with the actual measurement
                    [sensorParticleData] = obj.GrassSensor.measure(obj.Particles(1:3,i));
                    if (sensorParticleData.right == sensorData.right)
                        obj.Particles(4,i) = 0.8;
                    else
                        obj.Particles(4,i) = 0.2;    
                    end 
                end

                % Normalize weights and calculate effective number of particles
                obj.Particles(4,:) = obj.Particles(4,:) / sum(obj.Particles(4,:));
                N_eff = 1 / sum(obj.Particles(4,:).^2);

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
            sigma = std(obj.Particles(1:3,1:obj.NumParticles),0,2)';
        end
    end
    
    methods (Static)
        
        % Function to define the error of the line fit to the pathData
        function e = lineSegmentationError(X)
            % TODO add description
            x0 = X(:,1);
            x1 = X(:,end);
            v = x1 - x0;
            phi = atan2(v(2),v(1));
            m = length(X(1,:)) - 1; % Dont requied to test the error for x0, x1
            e = 0;
            for j=2:1:m
                vTest = X(:,j) - x0;
                phiTest = atan2(vTest(2),vTest(1));
                psi = phiTest - phi;
                e = e + (sin(psi)*norm(vTest))^2;
            end
            e = e/(m-1);
        end
        
        function result = compareHistory(DP,polyMap)
            % TODO: Add description
            
            % TODO: Change cmin to config script
            c_min = 0.3;
            
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
            if ((min(corr_tmp) - corr_min) > 0.1) && (corr_min < c_min)
                if corr_idx == 1
                    corr_idx = length(polyMap.x);
                end
                result.corr = corr;     % Debug
                result.foundPosition = true;
                result.position = [polyMap.x(corr_idx); polyMap.y(corr_idx)];
                result.phi = atan2(polyMap.y(corr_idx)-polyMap.y(corr_idx-1), ...
                                    polyMap.x(corr_idx)-polyMap.x(corr_idx-1));
            else
                result.foundPosition = false;
                result.corr = corr;
            end
%             valid = zeros(m,1);
%             for j=1:1:m
%                 [phiCum_map, LCum_map] = getCumOrientation(DP_map(:,1:(m+j)));
%                 corr = getCorrelation(phiCum_DP,LCum_DP,phiCum_map,LCum_map);
%                 if corr < c_min
%                     valid(j) = 1;
%                 end
%             end
%             if sum(valid) == 1 && (mod(find(valid),1) == 0) && (find(valid) > 2)
%                 result.foundPosition = true;
%                 idx = find(valid);
%                 result.position = [polyMap.x(idx); polyMap.y(idx)];
%                 result.phi = atan2(polyMap.y(idx)-polyMap.y(idx-1), ...
%                                     polyMap.x(idx)-polyMap.x(idx-1));
%             else
%                 result.foundPosition = false;
%             end
            
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

