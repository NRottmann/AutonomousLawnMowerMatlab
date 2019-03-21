function [X_aligned,E] = Compare2Map(estMap,map,method) 
    % Close map, thereby take the first position then take the
    % differences and search for minima
    X_closed = [estMap.x; estMap.y];
    X_map = [map.x; map.y];
    resultsComp = MapComparison(X_closed(1:2,:),X_map,method);
    X_aligned = resultsComp.X1;
    E = resultsComp.Comp;
end