% function visualize_binary_map(binary_map)
    % Get current axes
    hold on
    
    % Create X and Y coordinates for the surface
    % [X, Y] = meshgrid(1:size(binary_map,2), 1:size(binary_map,1));
    X = binary_map(:,1);
    Y = binary_map(:,2);
    % Create surface at specified height
    Z = ones(size(binary_map)) * 2;
    
    % Plot surface with binary map as color data
    plot(X, Y, Z, binary_map', ...
        'EdgeColor', 'red', ...
        'FaceColor', 'red', ...
        'FaceAlpha', 0.3);
    
    % Set colormap for binary visualization
    % colormap([0 0 0; 1 0 0]);  % Black for 0s, red for 1s
    
    % Keep axis properties
    % axis equal;
    grid off;
% end
