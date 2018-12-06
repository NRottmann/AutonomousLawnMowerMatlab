function [results] = realData(data,mode)
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
    %
    % Output:
    %   results:        Structure with the results. The content is
    %                   depending on the mode.
    
    if (mode == 1)
        poseGraphOptimization = PoseGraphOptimization(data(1:2,:));
        [matlabGraph,tutorialGraph,adjustedTutorialGraph,lagoGraph] = generateMap(poseGraphOptimization);
        results.matlabGraph = matlabGraph;
        results.tutorialGraph = tutorialGraph;
        results.adjustedTutorialGraph = adjustedTutorialGraph;
        results.lagoGraph = lagoGraph;
        results.odometryPose = data;
    else
        error('simulation: Wrong mode chosen!');
    end
end