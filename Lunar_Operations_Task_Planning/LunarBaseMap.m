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
            obj.paths = containers.Map('KeyType', 'char', 'ValueType', 'any');
            
            % Create and store 3D map
            obj.map_figure = create_3D_lunar_base();
            hold on
            scatter(50,50,2)
           
            % Initialize path connections
            % obj.initialize_paths();
        end
    end
end
