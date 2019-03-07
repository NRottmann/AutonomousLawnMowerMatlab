classdef NNCCPP
    % Neural Network approach for Complete Coverage Path Planning, the
    % approach is based on the (1) and (2)
    %
    % Date:     13.02.2019
    % Author:   Nils Rottmann (Nils.Rottmann@rob.uni-luebeck.de)
    %
    % (1): A Neural Network Approach to Complete Coverage Path Planning
    % (2): A Probabilistic Approach to High-Confidence Cleaning Guarantees for Low-Cost Cleaning Robots
    
    properties
        Map;                % Map as polygon
        ObstacleMap;        % Matrix [N,M] with values 0 (free) and 1 (obstacle)
        CoverageMap;        % Matrix [N,M] with values between 0 and 1 dependend on the probability if the area has been already covered
        NeuralActivity;     % Matrix [N,M] with values from -D (lower bound) and +B (upper bound)
        ExternalInput;      % Matrix [N,M] with values from -E to E
        
        A;                  % Passive decay rate
        B;                  % Upper bound
        D;                  % Lower bound
        E_max;              % Maximum Gain, for gradient
        E_min;              % Minimum Gain
        Alpha;              % Value for going through       
        T;                  % Treshold for when a field counts as covered   
        C;                  % Control gain
        Res;                % Resolution of the coverage map
        Dt;
        
        Cov;                % Covering Factor
        
        N;                  % x-dimension
        M;                  % y-dimension
    end
    
    methods
        function obj = NNCCPP(map,a,b,d,e_max,e_min,t,c,alpha,res)
            % Constructor of the class
            
            % Checks
            if ~isscalar(a)
                error('a not scalar!')
            end
            if ~isscalar(b)
                error('b not scalar!')
            end
            if ~isscalar(d)
                error('d not scalar!')
            end
            if ~isscalar(e_max)
                error('e_max not scalar!')
            end
            if ~isscalar(e_min)
                error('e_min not scalar!')
            end
            if ~isscalar(t)
                error('t not scalar!')
            end
            if ~isscalar(c)
                error('c not scalar!')
            end
            if ~isscalar(alpha)
                error('alpha not scalar!')
            end
            
            % Allocate map of the environment
            obj.Map = map;
            
            % Generate obstacle map, [N,M] Matrix initialized with zeros
            % when inside and ones when outside of a the environment
            obj.N = round((map.XWorldLimits(2) - map.XWorldLimits(1)) * res);
            obj.M = round((map.YWorldLimits(2) - map.YWorldLimits(1)) * res);
            obj.ObstacleMap = zeros(obj.N,obj.M);
            stepSize = 1/res;
            x_s = map.XWorldLimits(1) + 0.5*stepSize;
            for i=1:1:obj.N
                y_s = map.YWorldLimits(1) + 0.5*stepSize;
                for j=1:1:obj.M
                    if ~(inpolygon(x_s,y_s,map.x,map.y))
                        obj.ObstacleMap(i,j) = 1;
                    end
                    y_s = y_s + stepSize;
                end
                x_s = x_s + stepSize;
            end
            
            % Initialize Coverage map
            obj.CoverageMap = zeros(obj.N,obj.M);
            
            % Generate matrix for neural activity (required for the
            % planning method)
            obj.NeuralActivity = zeros(obj.N,obj.M);
            
            % Get parameters, TODO: Config?
            obj.A = a;
            obj.B = b;
            obj.D = d;
            obj.E_max = e_max;
            obj.E_min = e_min;
            obj.T = t;
            obj.Alpha = alpha;
            obj.C = c;          
            obj.Cov = 1;
            obj.Res = res;
            obj.Dt = 0.05;
        end
        
        function [obj,x] = planStep(obj,p,particles)
            % Plan the next step
            %
            % Input:
            %   obj:        Instance of the class
            %   p:          Current pose estimate
            %   particles:  Current distr. of particles
            %
            % Output:
            %   obj:        Instance of the class
            %   x:          Next target position
            
            % (1): Update coverage map based on the particle distribution
            obj = updateCoverageMap(obj,particles);
            % (2): Update external inputs
            coverageMap_tmp = obj.CoverageMap;
            idx_x = ceil((p(1) - obj.Map.XWorldLimits(1)) * obj.Res);
            idx_y = ceil((p(2) - obj.Map.YWorldLimits(1)) * obj.Res);
            coverageMap_tmp(idx_x,idx_y) = 1;
            obj.ExternalInput = obj.E_max*(ones(obj.N,obj.M) - coverageMap_tmp) - 100*obj.E_max*obj.ObstacleMap;
            % (3): Update neural activity
            obj = updateNeuralActivity(obj);
            % (4): Plan the next step
            x = planning(obj,p);
        end
        
        function obj = updateCoverageMap(obj,particles)
            % Update the coverage map based on the particle distribution
            n = length(particles(1,:));
            prob = zeros(obj.N,obj.M);
            for i=1:1:n
                idx_x = ceil((particles(1,i) - obj.Map.XWorldLimits(1)) * obj.Res);
                idx_y = ceil((particles(2,i) - obj.Map.YWorldLimits(1)) * obj.Res);
                if ((idx_x>=1 && idx_x<=obj.N) && (idx_y>=1 && idx_y<=obj.M))   % Check boundaries
                    prob(idx_x,idx_y) = prob(idx_x,idx_y) + 1/n;
                end
            end
            obj.CoverageMap = obj.CoverageMap + prob - prob.*obj.CoverageMap; 
        end
        
        function [obj] = updateNeuralActivity(obj)
            % Update the neural activity of the neurons based on the
            % shunting equation

            X = obj.NeuralActivity;
            Xp = X;                         % positiv neural activity
            Xp(Xp < 0) = 0;
            Xp_tmp = zeros(obj.N,obj.M);    % Generate weighted inhibitory inouts
            for i=1:1:obj.N
                for j=1:1:obj.M
                    for ii=-1:1:1
                        for jj=-1:1:1
                            iii = ii + i;
                            jjj = jj + j;
                            if ~(ii==0 && jj==0) && ((iii>=1 && iii<=obj.N) && (jjj>=1 && jjj<=obj.M))
                                Xp_tmp(i,j) = Xp_tmp(i,j) + norm([ii; jj]) * Xp(iii,jjj);
                            end
                        end
                    end
                end
            end
            Xp = Xp_tmp;
            
            I = obj.ExternalInput;
            Ip = I;                     % positiv external inputs
            Ip(Ip < 0) = 0;
            In = I;                     % negativ external inputs
            In(In > 0) = 0;
            In = -In;

            dX = -obj.A*X + (obj.B*ones(obj.N,obj.M) - X) .* (Ip + Xp) ...
                            - (obj.D*ones(obj.N,obj.M) + X) .* In;
                        
            obj.NeuralActivity = X + dX*obj.Dt;
        end
        
    	function x_next = planning(obj,pose)   
            % Choose the position to go next
            %
            % Input:
            %   obj:    Instance of the class
            %   pose:   Current pose estimate
            
            % Allocate neural activity
            X = obj.NeuralActivity;
            
            % Get the index according to the position of the vehicle
            idx_x = ceil((pose(1) - obj.Map.XWorldLimits(1)) * obj.Res);
            idx_y = ceil((pose(2) - obj.Map.YWorldLimits(1)) * obj.Res);
            
            dec = -inf;
            % Go through all neighbours and decide where to go next
            x_next = zeros(2,1);
            for i=-1:1:1
                for j=-1:1:1
                    if (X(idx_x,idx_y) <= X(idx_x+i,idx_y+j))    % && ~(i==0 && j==0)            % Do if value of cell is higher then initialPose and the cell is not the initial cell
                        if ((i+idx_x>=1 && i+idx_x<=obj.N) && (j+idx_y>=1 && j+idx_y<=obj.M))   % Check if out of bounds
                            ii = idx_x + i;
                            jj = idx_y + j;
                            dec_tmp  = X(ii,jj) - obj.C * abs(i*j);
                            if dec_tmp > dec
                                x_next(1) = ((ii-0.5)/obj.Res) + obj.Map.XWorldLimits(1);
                                x_next(2) = ((jj-0.5)/obj.Res) + obj.Map.YWorldLimits(1);
                                dec = dec_tmp;
                            end
                        end
                    end
                end
            end
        end
               
%         function pose_tmp = planning(obj,resolution,initialPose)   
%             % Allocate the Neural Activity
%             x = obj.NeuralActivity;
%             
%             % Get the index according to the position of the vehicle
%             i = floor(initialPose(1)*resolution + 1);
%             j = floor(initialPose(2)*resolution + 1);
%             
%             pose_tmp = initialPose;
%             dec = -inf;
%             % Go through all neighbours and decide where to go next
%             for ii=-1:1:1
%                 for jj=-1:1:1
%                     if (x(i,j) < x(i+ii,j+jj)) && ~(ii==0 && jj==0)     % do if value of cell is higher then initialPose and the cell is not the initial cell
%                         if ((ii+i>=1 && ii+i<=obj.N) && (jj+j>=1 && jj+j<=obj.M))  % Check if out of bounds
%                             iii = i+ii;
%                             jjj = j+jj;
%                             xn = (iii-0.5)/resolution;
%                             yn = (jjj-0.5)/resolution;
%                             phin = atan2(yn-initialPose(2),xn-initialPose(1));
%                             decn  = x(i+ii,j+jj) - obj.C * abs(ii*jj);
%                             % decn  = x(i+ii,j+jj); % + obj.C*(1-abs(phin-initialPose(3))/pi);
%                             if decn > dec
%                                 pose_tmp(1) = xn;
%                                 pose_tmp(2) = yn;
%                                 pose_tmp(3) = phin;
%                                 dec = decn;
%                             end
%                         end
%                     end
%                 end
%             end
%         end
        
        
%         function obj = updateExternalInput(obj,coverageMap)
%             % Function which updates the External Input based on the
%             % updated coverage map
%             
%             c_covered = 0;
%             
%             multiplier = 2;
%             
%             obj.CoverageMap = coverageMap;   
%             externalInput = multiplier*obj.ObstacleMap + obj.CoverageMap;
%             
%             Indices = [];
%             
%             for i=1:1:obj.N
%                 for j=1:1:obj.M
%                     if externalInput(i,j) >= multiplier     % Obstacle
%                         externalInput(i,j) = -(obj.E_max*obj.Cov);
%                     elseif externalInput(i,j) < obj.T       % not covered
%                         Indices = [Indices; i j];
%                     else                                    % covered
%                         c_covered = c_covered + 1;
%                         externalInput(i,j) = 0;
%                     end
%                 end 
%             end
%             
%             % Define the external Input for non-covered cells with  a
%             % gradient
%             if isempty(Indices)
%                 c_nonCovered = 0;
%                 x_min = 0;
%                 x_max = 0;
%             else
%                 c_nonCovered = length(Indices(:,1));
%                 x_min = min(Indices(:,1));
%                 x_max = max(Indices(:,1));
%             end
%             dX = x_max - x_min;
%             dE = obj.E_max - obj.E_min;
%             Emin = obj.E_min + ((obj.MaxDiffX - dX)/obj.MaxDiffX)*dE;
%             if dX ~= 0
%                 for i=1:1:c_nonCovered
%                     externalInput(Indices(i,1),Indices(i,2)) = (Emin + dE * (Indices(i,1) - x_min)/(dX)) * obj.Cov;
%                     % Vorfaktor: (1-externalInput(Indices(i,1),Indices(i,2)))
%                 end
%             else
%                 for i=1:1:c_nonCovered
%                     externalInput(Indices(i,1),Indices(i,2)) = obj.E_max * obj.Cov;
%                 end
%             end
%             
%             if c_nonCovered == 0
%                 obj.Cov = 1;
%             else
%                 obj.Cov = 1 + obj.Alpha * (c_covered+1)/(c_nonCovered);
%             end
%             
%             obj.ExternalInput = externalInput;
%         end
    end
end