function [SP] = findSPs(X,param)
            % Calculate the similar points between the data sets X1 and X2
            % 
            % input:
            %   X:  Cell array of size {2,1} containing matrices with
            %   points, size 2xN
            %
            %   param:          Parameter required for the algorithm
            %       m:  ...
            %       l_nh: ...
            %       c_min: ...
            % output:
            %   SP:             Incident matrix with similar points
            %
            
            % Get length and orientations
            for k=1:1:2
                M{k} = length(X{k}(1,:));      
                phi = zeros(M{k},1);
                l = zeros(M{k},1);
                for i=2:1:M{k}                        	% Go through all points
                    v = X{k}(:,i) - X{k}(:,i-1);
                    phi_tmp = atan2(v(2),v(1));
                    phi(i-1) = phi_tmp;               	% Orientation of line segments
                    l(i-1) = norm(v);                   % Length of line segments
                end
            
                % Accumulate Orientations
                phi_cumulated{k} = zeros(M{k},1);
                l_cumulated{k} = zeros(M{k},1);
                for i = 2:1:M{k}
                  delta_phi = phi(i) - phi(i-1);
                    % Regularization
                    if abs(delta_phi) > pi
                      if phi(i-1) > 0.0
                        delta_phi = delta_phi + 2*pi;
                      else
                        delta_phi = delta_phi - 2*pi;
                      end
                    end
                  phi_cumulated{k}(i) = phi_cumulated{k}(i-1) + delta_phi;  
                  l_cumulated{k}(i) = l_cumulated{k}(i-1) + l(i);
                end
            end
            
            % Calculate correlation
            corr = zeros(M{1},M{2});
            l_evaluation = linspace(-param.l_nh,param.l_nh,param.m);
            for i=1:1:M{1}
                l_cumulated_i = l_cumulated{1} - l_cumulated{1}(i);
                phi_cumulated_i = phi_cumulated{1} - phi_cumulated{1}(i);
                for j=1:1:M{2}
                    l_cumulated_j = l_cumulated{2} - l_cumulated{2}(j);
                    phi_cumulated_j = phi_cumulated{2} - phi_cumulated{2}(j);
                    for k=1:1:param.m
                        corr(i,j) = corr(i,j) ...
                            + (getOrientation(phi_cumulated_i,l_cumulated_i,l_evaluation(k)) ...
                                    - getOrientation(phi_cumulated_j,l_cumulated_j,l_evaluation(k)))^2;
                    end
                    corr(i,j) = corr(i,j) / param.m;
                end
            end
            
            % Calculate similar points
            l_min = param.l_nh;                         % Minimum Length
            l_max = l_cumulated{1}(end) - param.l_nh;   % Maximum Length
            SP = zeros(M{1},M{2});
            for i=1:1:M{1}
                if l_cumulated{1}(i) > l_min && l_cumulated{1}(i) < l_max
                    [pks, locs] = findpeaks(-corr(i,:));
                    locs(abs(pks) > param.c_min) = [];
                    SP(i,locs) = 1;
                end
            end
        end