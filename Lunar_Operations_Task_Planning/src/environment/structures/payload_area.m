function [coords, tangents] = payload_area(Cube_corner, base_z, len, breadth, height, path, path_height, track_width, Chamber_Opacity, Path_Opacity)
    x = Cube_corner(1);
    y = Cube_corner(2);
    z = base_z;

    % Define the grid for the mesh
    [X, Y] = meshgrid([x, x+len], [y, y+breadth]);
    Z_bottom = ones(size(X)) * z;
    Z_top = ones(size(X)) * (z + height);

    % Create surfaces with matching properties
    h1 = mesh(X, Y, Z_bottom);
    h2 = mesh(X, Y, Z_top);

    % Define the grid for the mesh
    [X, Z] = meshgrid([x, x+len], [z, z+height]);
    Y_bottom = ones(size(X)) * y;
    Y_top = ones(size(X)) * (y + breadth);

    % Create surfaces with matching properties
    h3 = mesh(X, Y_bottom, Z);
    h4 = mesh(X, Y_top, Z);

    % Create the bottom rail
    [X, Y] = meshgrid([x, x+len], [y, y+track_width]);
    Z_bottom = ones(size(X)) * (z + path_height-0.1);
    Z_top = ones(size(X)) * (z + path_height);
    h5 = mesh(X, Y, Z_bottom);
    h6 = mesh(X, Y, Z_top);

    x_points = x+track_width/2:0.1:(x+len-track_width/2);
    x_points = x_points(1,end:-1:1);
    coords1 = [x_points', repmat(y+track_width/2, length(x_points), 1), repmat((z + path_height-0.1), length(x_points), 1)];
    tangents1 = repmat([-1, 0, 0], length(x_points), 1);  % Points clockwise

    % Create the top left rail
    [X, Y] = meshgrid([x, x+(len-path)/2+track_width], [y+breadth-track_width, y+breadth]);
    Z_bottom = ones(size(X)) * (z + path_height-0.1);
    Z_top = ones(size(X)) * (z + path_height);
    h7 = mesh(X, Y, Z_bottom);
    h8 = mesh(X, Y, Z_top);

    x_points = x+track_width/2:0.1:(x+(len-path)/2+track_width/2);
    coords2 = [x_points', repmat(y+breadth-track_width/2, length(x_points), 1), repmat((z + path_height-0.1), length(x_points), 1)];
    tangents2 = repmat([1, 0, 0], length(x_points), 1);  % Points clockwise

    % Create the top right rail
    [X, Y] = meshgrid([x+(len-path)/2-track_width+path, x+len], [y+breadth-track_width, y+breadth]);
    Z_bottom = ones(size(X)) * (z + path_height-0.1);
    Z_top = ones(size(X)) * (z + path_height);
    h9 = mesh(X, Y, Z_bottom);
    h10 = mesh(X, Y, Z_top);

    x_points = x+(len-path)/2-track_width/2+path:0.1:(x+len-track_width/2);
    coords3 = [x_points', repmat(y+breadth-track_width/2, length(x_points), 1), repmat((z + path_height-0.1), length(x_points), 1)];
    tangents3 = repmat([1, 0, 0], length(x_points), 1);  % Points clockwise

    % Create the right rail
    [X, Y] = meshgrid([x+breadth-track_width, x+breadth], [y, y+breadth]);
    Z_bottom = ones(size(X)) * (z + path_height-0.1);
    Z_top = ones(size(X)) * (z + path_height);
    h11 = mesh(X, Y, Z_bottom);
    h12 = mesh(X, Y, Z_top);

    y_points = y+track_width/2:0.1:y+breadth-track_width/2;
    y_points = y_points(1,end:-1:1);
    coords4 = [repmat(x+len-track_width/2, length(y_points), 1), y_points', repmat((z + path_height-0.1), length(y_points), 1)];
    tangents4 = repmat([0, -1, 0], length(y_points), 1);   % Points clockwise

    % Create the left rail
    [X, Y] = meshgrid([x, x+track_width], [y, y+breadth]);
    Z_bottom = ones(size(X)) * (z + path_height-0.1);
    Z_top = ones(size(X)) * (z + path_height);
    h13 = mesh(X, Y, Z_bottom);
    h14 = mesh(X, Y, Z_top);

    y_points = y+track_width/2:0.1:y+breadth-track_width/2;
    coords5 = [repmat(x+track_width/2, length(y_points), 1), y_points', repmat((z + path_height-0.1), length(y_points), 1)];
    tangents5 = repmat([0, 1, 0], length(y_points), 1);   % Points clockwise

    complete_coords = [coords3; coords4; coords1; coords5; coords2];
    path_tangents = [tangents3; tangents4; tangents1; tangents5; tangents2];

    % Get total number of points
    n = size(complete_coords, 1);
    mid_point = floor(n/2);
    
    % Split coordinates and tangents
    coords = struct();
    tangents = struct();

    % First half (1 to n/2)
    coords.right = complete_coords(1:mid_point, :);
    tangents.right = path_tangents(1:mid_point, :);
    % Second half (n/2 to end, including overlap point)
    coords.left = complete_coords(end:-1:mid_point+1, :);
    tangents.left = path_tangents(end:-1:mid_point+1, :);

    % Apply properties to all mesh objects
    meshes_structure = [h1; h2; h3; h4];

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

    meshes_rail = [h5; h6; h7; h8; h9; h10; h11; h12; h13; h14];

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