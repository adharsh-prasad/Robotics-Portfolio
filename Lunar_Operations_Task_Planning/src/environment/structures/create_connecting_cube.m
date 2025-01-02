function [coords, tangents] = create_connecting_cube(Cube_corner, base_z, len, breadth, height, track_width,Chamber_Opacity, Path_Opacity)
    x = Cube_corner(1);
    y = Cube_corner(2);
    z = base_z;
    % Generate path coordinates with 0.25m spacing
    [coords, tangents] = deal(struct());

    % Define the grid for the mesh
    [X, Y] = meshgrid([x, x+len], [y, y+breadth]);
    Z_bottom = ones(size(X)) * z;
    Z_top = ones(size(X)) * (z + height);

    % Create surfaces with matching properties
    h1 = mesh(X, Y, Z_bottom);
    h2 = mesh(X, Y, Z_top);
    
    [Y, Z] = meshgrid([y, y+breadth], [z, z+height]);
    X_front = ones(size(Y)) * x;
    X_back = ones(size(Y)) * (x + len);
    h3 = mesh(X_front, Y, Z);
    h4 = mesh(X_back, Y, Z);

    % Create the left rail
    [X, Y] = meshgrid([x, x+track_width], [y, y+breadth]);
    Z_bottom = ones(size(X)) * (z + height-0.1);
    Z_top = ones(size(X)) * (z + height);
    h5 = mesh(X, Y, Z_bottom);
    h6 = mesh(X, Y, Z_top);

    y_points = y+track_width/2:0.1:y+breadth-track_width/2;
    coords.right = [repmat(x+track_width/2, length(y_points), 1), y_points', repmat((z + height-0.1), length(y_points), 1)];
    tangents.right = repmat([0, 1, 0], length(y_points), 1);   % Points up 

    % Create the right rail
    [X, Y] = meshgrid([x+len-track_width, x+len], [y, y+breadth]);
    Z_bottom = ones(size(X)) * (z + height-0.1);
    Z_top = ones(size(X)) * (z + height);
    h7 = mesh(X, Y, Z_bottom);
    h8 = mesh(X, Y, Z_top);

    coords.left = [repmat(x+len-track_width/2, length(y_points), 1), y_points', repmat((z + height-0.1), length(y_points), 1)];
    tangents.left = repmat([0, 1, 0], length(y_points), 1);   % Points up

    % Apply properties to all mesh objects
    meshes_structure = [h1; h2; h3; h4;];

    for h = meshes_structure'
        set(h, 'FaceColor', [1 1 1], ...
               'EdgeColor', 'none', ...
               'FaceAlpha', Chamber_Opacity, ...
               'FaceLighting', 'gouraud', ...
               'AmbientStrength', 0.3, ...
               'DiffuseStrength', 0.8, ...
               'SpecularStrength', 0.9, ...
               'SpecularExponent', 25);
    end

    meshes_rail = [h5; h6; h7; h8];

    for h = meshes_rail'
        set(h, 'FaceColor', [1 1 1], ...
               'EdgeColor', 'none', ...
               'FaceAlpha', Path_Opacity, ...
               'FaceLighting', 'gouraud', ...
               'AmbientStrength', 0.3, ...
               'DiffuseStrength', 0.8, ...
               'SpecularStrength', 0.9, ...
               'SpecularExponent', 25);
    end
end