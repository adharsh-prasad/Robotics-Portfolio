function final_path = Custom_Rail_Astar(paths, start_node, end_node)
    % Pre-compute end coordinates
    end_coords = paths(end_node).start_point;
    final_path = zeros(ceil(length(paths)/4), 1);  % Pre-allocate max possible size
    path_idx = 1;
    final_path(path_idx) = start_node;
    current_node = start_node;
    
    while true
        neighbours = paths(current_node).connections;
        n_neighbors = length(neighbours);
        scores = zeros(n_neighbors, 1);  % Pre-allocate scores array
        
        % Vectorized distance calculation
        for i = 1:n_neighbors
            scores(i) = manhattan_distance(paths(neighbours(i)).mid_point, end_coords);
        end
        
        [~, min_idx] = min(scores);
        current_node = neighbours(min_idx);
        path_idx = path_idx + 1;
        final_path(path_idx) = current_node;
        
        % Early termination check
        if (min(norm(paths(current_node).end_point - end_coords), ...
               norm(paths(current_node).start_point - end_coords)) < 0.5)
            break
        end
    end
    final_path = final_path(1:path_idx);  % Trim excess allocation
end


function dist = manhattan_distance(point1, point2)
    % Use Manhattan distance for grid-like rail system
    dist = abs(point1(1) - point2(1)) + abs(point1(2) - point2(2));
end