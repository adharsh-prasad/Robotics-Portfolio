function final_path = Custom_Rail_Astar(paths, start_node, end_node)
    % Initialize arrays
    current_node = start_node;
    end_coords = paths(end_node).start_point;
    final_path = start_node;
    while true
        neighbours = paths(current_node).connections;
        min_score = manhattan_distance(paths(neighbours(1)).mid_point, end_coords);
        min_neighbor = neighbours(1);
        for i = 2:length(neighbours)
            temp_score = manhattan_distance(paths(neighbours(i)).mid_point, end_coords);
            if temp_score<min_score
                min_neighbor = neighbours(i);
            end
        end
        current_node = min_neighbor;
        final_path = [final_path;min_neighbor];
        if ((norm(paths(current_node).end_point - end_coords) < 0.5) || norm(paths(current_node).start_point - end_coords) < 0.5)
            break
        end
    end
end

function dist = manhattan_distance(point1, point2)
    % Use Manhattan distance for grid-like rail system
    dist = abs(point1(1) - point2(1)) + abs(point1(2) - point2(2));
end


function path = reconstruct_numeric_path(came_from, current, node_list)
    % Initialize path with end node
    path = node_list(current);
    
    % Trace back through parents until reaching start node (where came_from is 0)
    while came_from(current) ~= 0
        current = came_from(current);
        path = [node_list(current), path];
    end
end
