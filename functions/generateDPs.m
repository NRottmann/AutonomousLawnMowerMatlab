function DP = generateDPs(data,param)
    % Function which prunes the given odometry data set
    %
    % Syntax:
    %       DP = generateDPs(data,param)
    %
    % input:
    %   data:   Odometry data (Matrix with 2 x N)
    %   param:  Parameters required by the algorithm
    %           l_min: minimum distance between two DPs
    %           e_max: maximum allowed line error 
    % output:
    %   DP:     Pruned odometry data points (Matrix with 2x M), we
    %           call them DPs (dominant points)

    % Check size, matrix has to be 2 x N
    if ~(length(data(:,1)) == 2)
        error('generateDPs: Size of data incorrect, should be 2 x N!')
    end
    if ~isscalar(param.l_min)
        error('generateDPs: param.l_min is not a scalar or does not exist!')
    end
    if ~isscalar(param.e_max)
        error('generateDPs: param.l_min is not a scalar or does not exist!')
    end
    
    N = length(data(1,:));

    DP = data(:,1);
    S = data(:,1);
    for i=2:1:N
        d = norm(DP(:,end) - data(:,i));
        if (d < param.l_min)
            S = [S data(:,i)];
        else
            S_tmp = [S, data(:,i)];
            e = errorLineFit(S);
            if (e < param.e_max)
                S = S_tmp;
            else
                DP = [DP, S(:,end)];
                S = [S(:,end),data(:,i)];
            end
        end
    end  
end

