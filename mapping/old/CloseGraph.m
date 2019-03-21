function [X_closed] = CloseGraph(X,A) 
    % Close map, thereby take the first position then take the
    % differences and search for minima
    
%     N = length(X(1,:)) - 1;
%     M = length(A(1,:)) - N;
%     
%     LC = zeros(M,2);
%     d = zeros(M,1);
%     a = zeros(M,1);
%     for i=1:1:M
%         [~,idx1] = min(A(:,N+i));
%         [~,idx2] = max(A(:,N+i));
%         d(i) = abs(idx2 - idx1);
%         a(i) = norm(X(1:2,idx1) - X(1:2,idx2));
%         LC(i,1) = idx1;
%         LC(i,2) = idx2;
%     end
%     while ~exist('X_closed') && ~isempty(d)
%         [~,idx] = min(d);
%         if a(idx) < 0.5
%             i1 = LC(idx,1);
%             i2 = LC(idx,2);
%             X_closed = [X(:,i1:i2-1), X(:,i1)];
%         else
%             a(idx) = [];
%             d(idx) = [];
%             LC{idx} = [];
%         end
%     end
%     
    
%     X_tmp = X(1:2,:) - X(1:2,1);
%     S_tmp = zeros(length(X_tmp(1,:)),1);
%     for i=1:1:length(X_tmp(1,:))
%         S_tmp(i) = norm(X_tmp(:,i));
%     end
%     [pks, locs] = findpeaks(-S_tmp);
%     for i=1:1:length(pks)   % The first would be the starting position, thus neglect
%         if abs(pks(i)) < 0.5
%             X_closed = [X(:,1:locs(i)-1),X(:,1)];
%             break
%         end
%     end
%     if ~exist('X_closed')
%         error('Could not generate closed loop map!')
%     end
    
j = 1;
while ~exist('X_closed')
    X_tmp = X(1:2,:) - X(1:2,j);
    S_tmp = zeros(length(X_tmp(1,:)),1);
    for i=1:1:length(X_tmp(1,:))
        S_tmp(i) = norm(X_tmp(:,i));
    end
    [pks, locs] = findpeaks(-S_tmp);
    for i=1:1:length(pks)   % The first would be the starting position, thus neglect
        if abs(pks(i)) < 0.5
            X_closed = [X(:,1:locs(i)-1),X(:,1)];
            break
        end
    end
    j = j + 1;
end

if ~exist('X_closed')
    error('Could not generate closed loop map!')
end

end