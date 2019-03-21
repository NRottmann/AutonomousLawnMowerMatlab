function [results] = MapComparison(X1,X2,mode)
    % Map comparion function
    %
    % Input:
    %   X1: Map 1 as points, 2 x N+1
    %   X2: Map 2 as points, 2 x N+1
    %   mode:  1: Horn Method, 2: Numerical approach
    % 
    % Output:
    %   result: comparison result
    
    if (mode == 1) || (mode == 3)
        % Generate Matrices for comparison
        X{1} = [X1(:,1:end-1) X1];
        X{2} = [X2(:,1:end-1) X2];

        % Find similar points between the maps
        out = get_config('mapping');
        param.l_nh = out.l_nh;
        param.c_min = out.c_min;
        param.m = out.M;
        [SP] = findSPs(X,param);

        % Pick the points and generate subsets
        N = length(SP(:,1));
        M = length(SP(1,:));
        X1_Subset = [];
        X2_Subset = [];
        for i=1:1:N
            for j=1:1:M
                if SP(i,j) == 1
                    X1_Subset = [X1_Subset, X{1}(:,i)];
                    X2_Subset = [X2_Subset, X{2}(:,j)];
                end
            end
        end

        % Use Horn Schema
        if ~isempty(X1_Subset) && ~isempty(X2_Subset)
            [regParams,~,~]=absor(X1_Subset,X2_Subset);
            X1 = regParams.R*X1 + regParams.t;
        end

        % Compare the maps
        Poly1 = polyshape(X1(1,:),X1(2,:),'Simplify',false);
        Poly2 = polyshape(X2(1,:),X2(2,:),'Simplify',false);
        Comp = area(xor(Poly1,Poly2))/area(Poly2);
    end
    if (mode == 2) || (mode == 3)
        if mode == 2
            X1 = X1 + (X2(:,1) - X1(:,1));
        end
        Poly1 = polyshape(X1(1,:),X1(2,:),'Simplify',false);
        Poly2 = polyshape(X2(1,:),X2(2,:),'Simplify',false);
        Comp = area(xor(Poly1,Poly2))/area(Poly2);
        % Holders
        trans_old = ones(3,1);
        J_old = zeros(3,1);
        % Params for numerical evaluation
        diff = 1;
        dx = 10^-(6);
        alpha = 1;
        while diff > 10^(-9)          
            % Generate numerical differetiate
            J = zeros(3,1);
            Poly1_x = translate(Poly1,dx,0);
            Poly1_y = translate(Poly1,0,dx);
            Poly1_phi = rotate(Poly1,dx);
            J(1) = (area(xor(Poly1_x,Poly2))/area(Poly2) - Comp) / dx;
            J(2) = (area(xor(Poly1_y,Poly2))/area(Poly2) - Comp) / dx;
            J(3) = (area(xor(Poly1_phi,Poly2))/area(Poly2) - Comp) / dx;
            Delta_F = J*Comp;
            alpha = (Delta_F)'*(J - J_old)/(norm(J - J_old)^2);
            trans = - alpha * Delta_F;
            for jj=1:1:3
                if ~isfinite(trans(jj)) || ~isreal(trans(jj)) || isnan(trans(jj))
                    trans(jj) = 0;
                end
            end
            Poly1 = rotate(Poly1,trans(3));
            Poly1 = translate(Poly1,trans(1),trans(2));
            Comp_tmp = area(xor(Poly1,Poly2))/area(Poly2)
            diff = Comp - Comp_tmp;
            Comp = Comp_tmp;
        end  
    end 
    X1 = [Poly1.Vertices' Poly1.Vertices(1,:)'];
    
    % Plot results
    figure(10)
    plot(X1(1,:),X1(2,:))
    hold on
    plot(X2(1,:),X2(2,:))
    legend('Map Estimate','Original Map')
    
    % Put into results
    results.X1 = X1;
    results.Comp = Comp;
end

