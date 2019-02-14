classdef SymbioticParticleFilter
    % Symbiotic Particle Filter class for autonomous lawn mower
    %
    % Methods:
    %   ParticleFilter(polyMap,p0,grassSensor,...
    %                       odometryModel,wallFollower,randomController,...
    %                       pdController,NNCCPP,mode)
    %   obj = update(obj,odometryData,sensorData)
    %   posParticles = getParticlePosition(obj)
    %   posParticles = getParticlePositionOrientationWeight(obj)
    %   rating = getParticleRating(obj)
    %   [mu,sigma] = getMeanVariance(obj)
    %
    % Nils Rottmann (Nils.Rottmann@rob.uni-luebeck.de)
    % 05.11.2018
    
    properties
        Particles;                      % Array of particles with every
                                        % row is one particle and contains
                                        % [x; y; phi; weight]
                                        
        DP;                             % 
        x_old;
        S;
        
        PoseMatch;
        
        PolyMap;                        % The map, represented by polygons
        
        PosRight;                       % Sensor position
        
        GrassSensor;                    % Instance of the class GrassSensor
        OdometryModel;                  % Instance of the class OdometryModel
        WallFollower;                   % Instance of the class WallFollower
        RandomController;               % Instance of the class RandomController
        PDController;                   % Instance of the class PDController
        NNCCPP;                         % Instance of the class NNCCPP
        
        Mode;                           % Operation mode of the particle filter
                                        % 1: Random Walk, 2: CCPP
                                        
        X_target;                       % Target position for CCPP
        
        GlobalLocalization;             % boolean, which tells us if global localization is required
        Relocalization;                 % True if Relocalization is required
        
        Counter;
        
        % Parameters
        W;
        Tr;
        L_min;
        E_max;
        U_min;
        C_min;
        N_P;                            % Number of particles used
    end
    
    methods
        function obj = SymbioticParticleFilter(polyMap,p0,grassSensor,...
                            odometryModel,wallFollower,randomController,...
                            pdController,NNCCPP,mode)
            % This is the constructor of the class
            % Syntax:
            %       obj = ParticleFilter(nParticles,polyMap,p0)
            % Input:
            %   polyMap:     	The map as polygon
            %   p0:             A start Pose for the particles with 
            %                   p = [x,y,phi,known]^T, if known = 0, we
            %                   start with global localization
            % Output:
            %   obj:            Intance of the class ParticleFilter
            
            % Check for correct dimensions
            % TODO: do dimensions check
            
            % Allocate Parameters
            obj.PolyMap = polyMap;
            obj.GrassSensor = grassSensor;
            obj.OdometryModel = odometryModel;
            obj.WallFollower = wallFollower;
            obj.RandomController = randomController;
            obj.PDController = pdController;
            obj.NNCCPP = NNCCPP;
            
            obj.Mode = mode;
            
            % Initialize variables
            obj.DP = zeros(2,1);
            obj.x_old = zeros(2,1);
            obj.S = zeros(2,1);
            obj.PoseMatch = zeros(3,1);
            
            % Sensor position
            out = get_config('Sensor');
            obj.PosRight =  out.posRight;
            
            % Parameters
            out = get_config('particleFilter');
            obj.W = out.w;
            obj.Tr = out.tr;
            obj.L_min = out.l_min;
            obj.E_max = out.e_max;
            obj.U_min = out.u_min;
            obj.C_min = out.c_min;
            obj.N_P = out.n_P;
            
            % Initialize Particles
            obj.Particles = zeros(4, obj.N_P);
            if p0(4) ~= 0
                for i = 1:1:obj.N_P
                    obj.Particles(:,i) = [p0(1); p0(2); ...
                                        p0(3); 1/obj.N_P];
                end
                obj.GlobalLocalization = false;
                obj.Relocalization = false;
                obj.X_target = [p0(1); p0(2)];
            else
                obj.GlobalLocalization = true;
                obj.Relocalization = true;
            end
        end
        
        function [obj,u] = update(obj,sensorData,odometryData,pEst)
            % This is the update function of the class which holds the
            % symbiotic particle filter algorithm
            % Syntax:
            %       obj = update(obj,odometryData,sensorData)
            % Input:
            %   obj:            Object of the Particle Filter class
            %   odometryData:   deltaR1, deltaT, deltaR2
            %   sensorData:     data from the sensor estimation
            %   pEst:           Pose estimate
            % Output:
            %   obj:            Instance of the class ParticleFilter
            
            %% Set odometry data and estimate sensor position
            obj.OdometryModel.DeltaR1 = odometryData.DeltaR1;
            obj.OdometryModel.DeltaT = odometryData.DeltaT;
            obj.OdometryModel.DeltaR2 = odometryData.DeltaR2;
            
            %% The algorithm
            result.foundPosition = false;
            if obj.GlobalLocalization
                % Get sensor position estimate and generate control signals
                R = [cos(pEst(3)) -sin(pEst(3)); sin(pEst(3)) cos(pEst(3))];
                x_new = pEst(1:2) + R*obj.PosRight;
                [obj.WallFollower,u] = obj.WallFollower.wallFollowing(sensorData);
                % Check whether the boundary has been found
                if obj.WallFollower.Mode == 0   % dont save anything
                    obj.DP = x_new;
                    obj.S = x_new;
                    obj.x_old = x_new;
                else                            % Do the globacl localization
                    L = norm(x_new - obj.DP(:,end));
                    if (L < obj.L_min)
                        obj.S = [obj.S, x_new];
                    else
                        S_tmp = [obj.S, x_new];
                        e = errorLineFit(S_tmp);
                        if (e < obj.E_max)
                            obj.S = S_tmp;
                        else    % If new DP, check if we already found a pose estimate
                            obj.DP = [obj.DP, obj.S(:,end)];
                            obj.S = [obj.S(:,end), x_new];
                            % Check the length and compare with
                            % circumeference of the map
                            l_DP = 0;
                            for i=2:1:length(obj.DP(1,:))
                                l_DP = l_DP + norm(obj.DP(:,i) - obj.DP(:,i-1));
                            end
                            if l_DP > obj.U_min*obj.PolyMap.Circumference
                                % Delete last DP if too long
                                if l_DP > obj.PolyMap.Circumference         
                                    obj.DP(:,1) = [];
                                end
                                compParam.c_min = obj.C_min;
                                result = SymbioticParticleFilter.compare(obj.DP,obj.PolyMap,compParam);
                            end
                        end
                    end
                    if result.foundPosition
                        obj.GlobalLocalization = false;
                        % Initialize Particles, TODO: Optimize this, maybe
                        % use Machine Learning, Reinforcement Learning
                        % Transform from estimated sensor position to
                        % actual location of the robot
                        R = [cos(result.phi) -sin(result.phi); ...
                                    sin(result.phi) cos(result.phi)];
                        result.position = result.position - R*obj.PosRight;
                        for i = 1:1:obj.N_P
                            x = normrnd(result.position(1),0.2);
                            y = normrnd(result.position(2),0.2);
                            phi = normrnd(result.phi,0.5);
                            obj.Particles(:,i) = [x; y; phi; 1/obj.N_P];
                        end
                        obj.PoseMatch = [result.position; result.phi];
                        obj.X_target = result.position;
                    end
                end
            else
                % Generate Control signals
                if (obj.Mode == 1)      % Random Walk Mode
                    [obj.RandomController,u] = obj.RandomController.randomControl(sensorData,odometryData);
                elseif (obj.Mode == 2)      % CCPP
                    % Check if relocalization is required
                    std_est = norm(std(obj.Particles(1:3,1:obj.N_P),0,2));
                    if (std_est > 0.6)
                        obj.Relocalization = true;
                    elseif (std_est < 0.2)
                        obj.Relocalization = false;
                    end
                    if (obj.Relocalization)
                        [obj.WallFollower,u] = obj.WallFollower.wallFollowing(sensorData);
                        obj.X_target = [mean(obj.Particles(1,:)); mean(obj.Particles(2,:))];
                    else
                        est_pose = [mean(obj.Particles(1,:)); mean(obj.Particles(2,:)); mean(obj.Particles(3,:))];
                        [obj.NNCCPP,obj.X_target] = planStep(obj.NNCCPP,est_pose,obj.Particles);
                        [obj.PDController,u] = pdControl(obj.PDController,est_pose(1:2),...
                                                            [0;0],obj.X_target,[0;0],est_pose(3),0);
                    end
                end
                %% Update
                % For all Particles, update them according to odometryData and
                % allocate weights according to measurement data
                for i = 1:1:obj.N_P
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
                treshResample =  out.tr;
                if (N_eff < treshResample * obj.N_P)
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
                    % Allocate Particles and set new N_P
                    obj.Particles = tempParticles;
                end 
            end
        end
             
       % TODO: Check which functions are really required
        function posParticles = getParticlePosition(obj)
            % Function to give back the positions of the particles
            posParticles = obj.Particles(1:2,1:obj.N_P);
        end
        
        function posParticles = getParticlePositionOrientationWeight(obj)
            % Function to get the positions of the particles with the 
            % weight and the orientation 
            posParticles = obj.Particles(:,1:obj.N_P);
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
            mu = mean(obj.Particles(1:3,1:obj.N_P),2)';
            sigma = std(obj.Particles(1:3,1:obj.N_P),0,2)';
        end
    end
    
    methods (Static)
        
        function result = compare(DP,polyMap,param)
            % Function which compares the DPs (generated from the odometry
            % data) with the polygonal map
            %
            % Input:
            %   DP:         Dominant points
            %   polyMap:    poly map representation
            %   param:      parameter
            %                   c_min
            % Output:
            %   result:     structure with results
            
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
            if ((min(corr_tmp) - corr_min) > 0.1) && (corr_min < param.c_min)
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

