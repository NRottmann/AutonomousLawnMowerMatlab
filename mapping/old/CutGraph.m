function [X_cutted,A_cutted] = CutGraph(X,A) 
    % Deletes the beginning and the end of the graph
    % Clear beginning and end
    [~,idx1] = min(A(:,length(X(1,:))+1));      % First loop closing constraint
    [~,idx2] = max(A(:,end));                   % Last loop closing constraint
    X_cutted = X(:,idx1:idx2);
    A_cutted = A(idx1:idx2,:);
    A_cutted(:,idx2:length(X(1,:))-1) = [];
    A_cutted(:,1:idx1-1) = [];
end