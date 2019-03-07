function [results] = realData(data,mode,param)
    % This function simulates the robot
    %
    % Syntax:
    %       [results] = simulation(polyMap,mode)
    %
    % Input:
    %   data:           The real data set, depending on the mode
    %   Mode:           There are different simulation modes. Please choose
    %                   one of the following
    %                   1: mapping
    %                   2: mapping and comparison, requires param.map
    %
    % Output:
    %   results:        Structure with the results. The content is
    %                   depending on the mode.
    
    if (mode == 1) || (mode == 2)
        poseGraphOptimization = PoseGraphOptimization(data(1:2,:));
        [X,A] = generateMap(poseGraphOptimization);
        [X_cutted,A_cutted] = CutGraph(X,A);
        [X_closed] = CloseGraph(X_cutted,A_cutted);
        results.X = X;
        results.X_cutted = X_cutted;
        results.X_closed = X_closed;
        results.odometryPose = data;
        if (mode == 2)
            [X_aligned,E] = Compare2Map(X_closed,param.map,param.compMethod);
            results.X_aligned = X_aligned;
            results.E = E;
        end
    else
        error('simulation: Wrong mode chosen!');
    end
end