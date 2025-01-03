function final_path = Constrained_Rail_Astar(paths, start_node, end_node)
    % Initialize data structures
    tic
    open_set = {start_node};
    came_from = containers.Map('KeyType', 'char', 'ValueType', 'char');
    g_score = containers.Map('KeyType', 'char', 'ValueType', 'double');
    f_score = containers.Map('KeyType', 'char', 'ValueType', 'double');
    
    % Set initial scores
    g_score(start_node) = 0;
    f_score(start_node) = manhattan_distance(paths(start_node).coordinates(end,:), ...
                                          paths(end_node).coordinates(1,:));
    
    while ~isempty(open_set)
        current = get_lowest_fscore(open_set, f_score);
        
        if strcmp(current, end_node)
            final_path = reconstruct_path(came_from, current);
            return;
        end
        
        open_set = setdiff(open_set, {current});
        
        % Process connections from path dictionary
        for i = 1:length(paths(current).connections)
            neighbor = paths(current).connections{i};
            
            % Path segment length as g_score
            tentative_g_score = g_score(current) + ...
                size(paths(current).coordinates, 1) * 0.25;  % 0.25m spacing
            
            if ~isKey(g_score, neighbor) || tentative_g_score < g_score(neighbor)
                came_from(neighbor) = current;
                g_score(neighbor) = tentative_g_score;
                f_score(neighbor) = g_score(neighbor) + ...
                    manhattan_distance(paths(neighbor).coordinates(end,:), ...
                                    paths(end_node).coordinates(1,:));
                
                if ~ismember(neighbor, open_set)
                    open_set{end+1} = neighbor;
                end
            end
        end
    end
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

function path = reconstruct_path(came_from, current)
    % Initialize path with end node
    path = {current};
    
    % Trace back through parents until reaching start node
    while isKey(came_from, current)
        current = came_from(current);
        path = [{current}, path];  % Add parent to front of path
    end
end
