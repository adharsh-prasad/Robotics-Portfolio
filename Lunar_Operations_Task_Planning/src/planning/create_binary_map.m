function binary_map = create_binary_map(paths, fig,resolution)
    % Get map dimensions from environment parameters
    x_max = fig.Children.XLim(2);
    y_max = fig.Children.YLim(2);
    
    % Create grid with specified resolution
    grid_x = ceil(x_max/resolution);
    grid_y = ceil(y_max/resolution);
    binary_map = zeros(grid_x, grid_y);
    
    % Mark rail paths as occupied (1)
    path_key = keys(paths);
    for j = path_key'
        coords = paths(j).coordinates;
        % Convert continuous coordinates to grid cells
        grid_points = ceil(coords/resolution);
        % Add path width consideration
        for i = 1:size(grid_points,1)
            binary_map(max(1,grid_points(i,1)-1):min(grid_x,grid_points(i,1)+1), ...
                      max(1,grid_points(i,2)-1):min(grid_y,grid_points(i,2)+1)) = 1;
        end
    end
end
