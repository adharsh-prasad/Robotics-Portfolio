function [binary_map, map_dims] = create_binary_map(paths, fig, resolution)
    % Get map dimensions from figure axis limits
    ax = fig.Children;
    x_max = ax.XLim(2);
    y_max = ax.YLim(2);
    
    % Create grid with specified resolution
    grid_x = round(x_max / resolution);
    grid_y = round(y_max / resolution);
    binary_map = ones(grid_y, grid_x);  % Initialize with obstacles (1s)
    
    % Mark paths as traversable (0s)
    path_key = keys(paths);
    for j = 1:length(path_key)
        coords = paths(path_key{j}).coordinates;
        binary_map = fill_binary_map(coords, resolution, binary_map);
    end
    
    % Return the dimensions of the map
    map_dims = [grid_y, grid_x];
end

function binary_map = fill_binary_map(coords, resolution, binary_map)
    [grid_y, grid_x] = size(binary_map);
    radius = 2 ; % 1 unit radius in grid coordinates
    
    for i = 1:size(coords, 1)
        % Calculate center of the circle in grid coordinates
        center_x = coords(i, 1) / resolution;
        center_y = coords(i, 2) / resolution;
        
        % Create a grid of coordinates
        [X, Y] = meshgrid(1:grid_x, 1:grid_y);
        
        % Calculate distances from center to all points
        distances = sqrt((X - center_x).^2 + (Y - center_y).^2);
        
        % Find all points within the circle
        circle_mask = distances <= radius;
        
        % Set all these points to 0 in the binary map
        binary_map(circle_mask) = 0;
    end
end