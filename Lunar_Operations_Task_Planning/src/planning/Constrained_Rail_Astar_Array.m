function final_path = Constrained_Rail_Astar_Array(paths, start_node, end_node)
    % Initialize arrays
    node_list = keys(paths);
    node_to_idx = containers.Map(node_list, 1:length(node_list));
    n_nodes = length(node_list);
    g_score = inf(n_nodes, 1);
    f_score = inf(n_nodes, 1);
    came_from = zeros(n_nodes, 1);
    
    % Get indices
    start_idx = node_to_idx(start_node);
    end_idx = node_to_idx(end_node);
    
    % Initialize start node
    g_score(start_idx) = 0;
    f_score(start_idx) = manhattan_distance(paths(start_node).end_point, paths(end_node).start_point);
    
    open_set = start_idx;
    end_coords = paths(end_node).start_point;

    while ~isempty(open_set)
        % Find node with minimum f_score in open_set
        [~, min_idx] = min(f_score(open_set));
        current = open_set(min_idx);
        
        % Process neighbors
        current_node = node_list{current};

        % Check if current node's end point is close to target
        if (norm(paths(current_node).end_point - end_coords) < 0.5) || norm(paths(current_node).start_point - end_coords) < 0.5
            final_path = reconstruct_numeric_path(came_from, current, node_list);
            return;
        end
        
        % Remove current node from open_set
        open_set(min_idx) = [];
                
        for i = 1:length(paths(current_node).connections)
            neighbor_str = paths(current_node).connections{i};
            neighbor = node_to_idx(neighbor_str);
            
            tentative_g_score = g_score(current) + paths(current_node).distance;
            
            if tentative_g_score < g_score(neighbor)
                came_from(neighbor) = current;
                g_score(neighbor) = tentative_g_score;
                f_score(neighbor) = tentative_g_score + manhattan_distance(...
                    paths(node_list{neighbor}).end_point, ...
                    paths(end_node).start_point);
                
                if ~ismember(neighbor, open_set)
                    open_set = [open_set; neighbor];
                end
            end
        end
    end
    
    % If no path found
    final_path = {};
end



function dist = manhattan_distance(point1, point2)
    % Use Manhattan distance for grid-like rail system
    dist = abs(point1(1) - point2(1)) + abs(point1(2) - point2(2));
end

function current = get_lowest_fscore(open_set, f_score)
    min_score = inf;
    current = '';
    for i = 1:length(open_set)
        node = open_set{i};
        if f_score(node) < min_score
            min_score = f_score(node);
            current = node;
        end
    end
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