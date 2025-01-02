classdef LunarBaseMap < handle
    properties
        map_figure      % Store the 3D visualization
        paths          % Dictionary of path connections
        dimensions     % Map dimensions
        node_coords    % Physical coordinates of nodes
    end

    methods
        function obj = LunarBaseMap()
            % Initialize paths dictionary
            % For your map nodes
            obj.paths = dictionary();
            obj.paths('SA31') = struct('connections', {'J32', 'J33'}, ...
                      'coordinates', [n x 3], ...    % Path points
                      'tangents', [n x 3], ...       % Unit vectors for orientation
                      'headings', [n x 1]);          % Angles in radians

            % Create and store 3D map
            obj.map_figure = create_3D_lunar_base();
            hold on
            scatter(50,50,2)
        end
    end
end
