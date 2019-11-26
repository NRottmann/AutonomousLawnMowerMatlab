classdef NNCCPP
    % Neural Network approach for Complete Coverage Path Planning, the
    % approach is based on (1)
    %
    % Date:     06.03.2019
    % Author:   Nils Rottmann (Nils.Rottmann@rob.uni-luebeck.de)
    %
    % (1): A Neural Network Approach to Complete Coverage Path Planning
    
    properties
        % Storage variables
        PolyMap;            % the map struct
        ObstacleMap;        % Matrix [N,M] with ones if obstacle and zero if lawn
        NeuralActivity;     % Matrix [N,M] with values from -D (lower bound) and +B (upper bound)
        ExternalInput;      % Matrix [N,M] with values from -E to E
        N;                  % Total number of cells x-dimension
        M;                  % Total number of cells y-dimension
        TargetPosition;     % Current target position
        Gradient;           % Matrix [N,M] resembling the gradient over the map
        Filter;             % Matrix [3,3] used to filter the weights for the shunting equation
        
        % Parameter
        Resolution;         % Resolution of coverage map
        A;                  % Passive decay rate
        B;                  % Upper bound
        D;                  % Lower bound
        E;                  % Gain
        C;                  % Control gain  
        Threshhold;          % Threshhold for the coverageMap
        G;                  % descent of the gradient

        Dt;                 % Step time   
    end
    
    methods
        function obj = NNCCPP()
            % Constructor
            out = get_config('planning');
            obj.A = out.a;
            obj.B = out.b;
            obj.D = out.d;
            obj.E = out.e;
            obj.C = out.c;
            obj.Threshhold = out.threshhold;
            obj.Dt = out.dt;
            obj.G = out.g;
            
            out = get_config('coverageMap');
            obj.Resolution = out.resolution;
            
            obj.Filter = [sqrt(2) 1 sqrt(2); 1 0 1; sqrt(2) 1 sqrt(2)];
            
        end
        
        function obj = initializeNeuralNet(obj,polyMap)
            % This method initializes the neural
            % on the given polygon map
            % Syntax:
            %       obj = initializeNeuralNet(obj,polyMap)
            % Input:
            %   polyMap:        A polygon map of the environment
            
            % Generate maps
            obj.PolyMap = polyMap;
            obj.N = round((polyMap.XWorldLimits(2) - polyMap.XWorldLimits(1)) * obj.Resolution);
            obj.M = round((polyMap.YWorldLimits(2) - polyMap.YWorldLimits(1)) * obj.Resolution);
            obj.ObstacleMap = zeros(obj.N,obj.M);
            stepSize = 1/obj.Resolution;
            x_s = polyMap.XWorldLimits(1) + 0.5*stepSize;
            for i=1:1:obj.N
                y_s = polyMap.YWorldLimits(1) + 0.5*stepSize;
                for j=1:1:obj.M
                    if ~(inpolygon(x_s,y_s,polyMap.x,polyMap.y))
                        obj.ObstacleMap(i,j) = 1;
                    end
                    y_s = y_s + stepSize;
                end
                x_s = x_s + stepSize;
            end
            
            obj.Gradient = ones(obj.N, obj.M)-obj.ObstacleMap;
            for i = 1:1:obj.M
                obj.Gradient(:,i) = obj.Gradient(:, i) .* (obj.G*(obj.M - i - (polyMap.YWorldLimits(2) - polyMap.YWorldLimits(1))));
            end
            obj.Gradient = obj.Gradient + obj.M*obj.G;
            
            % Initialize matrix for neural activity
            obj.NeuralActivity = zeros(obj.N,obj.M);
            
            % Initialize current target position
            obj.TargetPosition = zeros(2,1);
        end
        
        function [obj,x] = planStep(obj,pose,coverageMap)
            % Plan the next step
            %
            % Input:
            %   pose:           Current pose of the robot
            %   coverageMap:    Current coverage map for the robot
            %
            % Output:
            %   obj:        Instance of the class
            %   x:          Next target position

            % Update external inputs
            coverageMap_tmp = coverageMap;
            idx_x = ceil((pose(1) - obj.PolyMap.XWorldLimits(1)) * obj.Resolution);
            idx_y = ceil((pose(2) - obj.PolyMap.YWorldLimits(1)) * obj.Resolution);
            if ((idx_x>=1 && idx_x<=obj.N) && (idx_y>=1 && idx_y<=obj.M))
                coverageMap_tmp(idx_x,idx_y) = 1;
            end
            coverageMap_tmp(coverageMap_tmp >= obj.Threshhold) = 1;
            coverageMap_tmp(coverageMap_tmp < obj.Threshhold) = 0;
            obj.ExternalInput = obj.E.*(ones(obj.N,obj.M) - coverageMap_tmp) - 2*obj.E.*obj.ObstacleMap;
            obj.ExternalInput(obj.ExternalInput < -obj.E) = -obj.E;
            
            % Update neural activity
            obj = updateNeuralActivity(obj);
            % Plan the next step
            obj = planning(obj,pose);
            x = obj.TargetPosition;
        end
        
        function [obj] = updateNeuralActivity(obj)
            % Update the neural activity of the neurons based on the
            % shunting equation

            X = obj.NeuralActivity;
            Xp = X;                         % positiv neural activity
            Xp(Xp < 0) = 0;

            Xp_tmp = imfilter(Xp, obj.Filter); % generate weighted inhibitory inputs for the shunting equation
            Xp = Xp_tmp;
            
            I = obj.ExternalInput;
            Ip = I;                     % positiv external inputs
            Ip(Ip < 0) = 0;
            In = I;                     % negativ external inputs
            In(In > 0) = 0;
            In = -In;

            % calculate the change for the neural activity
            dX = -obj.A*X + (obj.B*ones(obj.N,obj.M) - X) .* (Ip + Xp) ...
                            - (obj.D*ones(obj.N,obj.M) + X) .* In;
                        
            obj.NeuralActivity = X + dX*obj.Dt;
        end
        
    	function obj = planning(obj,pose)
            % Choose the position to go next
            %
            % Input:
            %   obj:    Instance of the class
            %   pose:   Current pose estimate
            
            % Allocate neural activity
            X = obj.NeuralActivity;
            X = X .* obj.Gradient;
            
            % Get the index according to the position of the vehicle
            idx_x = ceil((pose(1) - obj.PolyMap.XWorldLimits(1)) * obj.Resolution);
            idx_y = ceil((pose(2) - obj.PolyMap.YWorldLimits(1)) * obj.Resolution);
            
            dec = -inf;
            % Go through all neighbours and decide where to go next
            for i=-1:1:1
                for j=-1:1:1
                    if ((i+idx_x>=1 && i+idx_x<=obj.N) && (j+idx_y>=1 && j+idx_y<=obj.M))   % Check if out of bounds
                        if (X(idx_x,idx_y) <= X(idx_x+i,idx_y+j)) && ~(i==0 && j==0)           % Do if value of cell is higher then initialPose and the cell is not the initial cell
                            ii = idx_x + i;
                            jj = idx_y + j;
                            % calculate orientation difference for control
                            % input
                            orientation = mod(pose(3), 2*pi);
                            orientationDiff = abs(orientation - orientate(i,j));
                            if orientationDiff > pi
                                orientationDiff = 2*pi - orientationDiff;
                            end
                            % calculate weighted control input
                            dec_tmp  = X(ii,jj) - obj.C * orientationDiff;
                            % update control input
                            if dec_tmp > dec
                                obj.TargetPosition(1) = ((ii-0.5)/obj.Resolution) + obj.PolyMap.XWorldLimits(1);
                                obj.TargetPosition(2) = ((jj-0.5)/obj.Resolution) + obj.PolyMap.YWorldLimits(1);
                                dec = dec_tmp;
                            end
                        end
                    end
                end
            end
        end
    end
end