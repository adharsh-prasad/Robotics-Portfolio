function fig = create_3D_lunar_base()
    close all
    
    % Define base parameters
    Sr = 4;%Superadobe_radius
    So = 2;%Superadobe_offset
    Dbs = 2*Sr + So;%Distance_btw_superadobe
    Mcel = 2;%Main_chamber_end_length
    Mcw = 2;%Main_chamber_width
    Msil = 4;%Mainpath_Superadobe_intersection_length
    Msib = 2;%Mainpath_Superadobe_intersection_breadth
    Msih = 2;%Mainpath_Superadobe_intersection_height
    Sd = 4;%Supplies_dim
    Psm = 6;%Payload area dimensions
    Psh = 3;%Payload area height
    Dm = 1/2*Psm + Sr + So;%Distance_in_middle
    tw = 0.3; %Track width

    %Axis dimensions
    map_x_length = 2*(Sr + 2*Dbs + 1*Dm + Msib/2);
    map_y_length = (Sr*4 + Msil*2 + Mcw);
    x_axis = 5*(floor(map_x_length/5) + 1);
    y_axis = 5*(floor(map_y_length/5) + 1);
    z_axis = 5*(floor(Psm/5) + 1);
    map_x_axis_offset = (x_axis - map_x_length)/2;
    map_y_axis_offset = (y_axis - map_y_length)/2;

    % Create fullscreen figure with black background
    fig = figure('Units', 'normalized', 'Position', [0 0 1 1], ...
                'Color', 'black', 'MenuBar', 'none', ...
                'ToolBar', 'none', 'WindowState', 'fullscreen');
    ax = axes('Color', 'black', 'Position', [0 0 1 1]);
    hold on;
    Chamber_Opacity = 0.3;
    Path_Opacity = 0.4;
    
    % Set view for top-down perspective with slight angle
    view(45, 45);
   
    campos([x_axis+50 -y_axis 50]);
    camtarget([x_axis/2 y_axis/2 0]);

    % Set up main lighting
    lighting gouraud;
    camlight('headlight');

    % Add additional lights for better depth perception
    camlight('right');
    camlight('left');

    % Customize light properties
    h = findobj(gca,'Type','light');
    set(h,'Style','infinite');
    set(h,'Color',[1 1 1]); % White light

    
    % Define superadobe centers in a more compact arrangement
    Superadobe_centers = [];
    Map_Start_Point = [map_x_axis_offset, y_axis/2];
    
    %top left 3
    new_sr = sqrt(Sr^2 - Msih^2);
    temp = Map_Start_Point + [Sr + Msib/2, Mcw/2 + Msil + new_sr*cos(asin(Msib/(2*new_sr)))];
    Superadobe_centers = [Superadobe_centers;temp];
    Superadobe_centers = [Superadobe_centers;temp + [Dbs,0]];
    Superadobe_centers = [Superadobe_centers;temp + 2*[Dbs,0]];

    %top left 2
    Superadobe_centers = [Superadobe_centers;temp + 2*[Dm,0] + 2*[Dbs,0]];
    Superadobe_centers = [Superadobe_centers;temp + 2*[Dm,0] + 4*[Dbs,0]];

    %bottom left 2
    new_sr = sqrt(Sr^2 - Msih^2);
    temp = Map_Start_Point + [Sr + Msib/2, -(Mcw/2 + Msil + new_sr*cos(asin(Msib/(2*new_sr))))];
    Superadobe_centers = [Superadobe_centers;temp];
    Superadobe_centers = [Superadobe_centers;temp + 1*[Dbs,0]];
    Superadobe_centers = [Superadobe_centers;temp + 2*[Dbs,0]];

    %top right 3
    Superadobe_centers = [Superadobe_centers;temp + 2*[Dm,0] + 2*[Dbs,0]];
    Superadobe_centers = [Superadobe_centers;temp + 2*[Dm,0] + 3*[Dbs,0]];
    Superadobe_centers = [Superadobe_centers;temp + 2*[Dm,0] + 4*[Dbs,0]];

    % Create domes
    for i = 1:size(Superadobe_centers, 1)
        create_dome(Superadobe_centers(i,1), Superadobe_centers(i,2), Sr, Msib, Msih, tw, y_axis, Chamber_Opacity, Path_Opacity);
    end

    % Define superadobe centers in a more compact arrangement
    Intersection_centers = [];
    Map_Start_Point = [map_x_axis_offset, y_axis/2];

    %top left 3
    temp = Map_Start_Point + [Sr, Mcw/2];
    Intersection_centers = [Intersection_centers;temp];
    Intersection_centers = [Intersection_centers;temp + [Dbs,0]];
    Intersection_centers = [Intersection_centers;temp + 2*[Dbs,0]];

    %top left 2
    Intersection_centers = [Intersection_centers;temp + 2*[Dm,0] + 2*[Dbs,0]];
    Intersection_centers = [Intersection_centers;temp + 2*[Dm,0] + 4*[Dbs,0]];

    %bottom left 2
    temp = Map_Start_Point + [Sr, -Mcw/2-Msil];
    Intersection_centers = [Intersection_centers;temp];
    Intersection_centers = [Intersection_centers;temp + 1*[Dbs,0]];
    Intersection_centers = [Intersection_centers;temp + 2*[Dbs,0]];

    %top right 3
    Intersection_centers = [Intersection_centers;temp + 2*[Dm,0] + 2*[Dbs,0]];
    Intersection_centers = [Intersection_centers;temp + 2*[Dm,0] + 3*[Dbs,0]];
    Intersection_centers = [Intersection_centers;temp + 2*[Dm,0] + 4*[Dbs,0]];

    %payload area
    Intersection_centers = [Intersection_centers;temp + [2*Dbs+Dm,0]];

    % Create intersections
    for i = 1:size(Intersection_centers, 1)
        create_connecting_cube(Intersection_centers(i,:), 0, Msib, Msil, Msih, tw, Chamber_Opacity, Path_Opacity)
    end

    % Create main path
    create_main_path(Map_Start_Point - [0 Mcw/2], 0, Sr, Msib, Msih, tw, Chamber_Opacity, Path_Opacity)
    create_main_path(Map_Start_Point + [(Sr+Msib) -Mcw/2], 0, Dbs-Msib, Msib, Msih, tw, Chamber_Opacity, Path_Opacity)
    create_main_path(Map_Start_Point + [(Sr+Msib+Dbs) -Mcw/2], 0, Dbs-Msib, Msib, Msih, tw, Chamber_Opacity, Path_Opacity)
    create_main_path(Map_Start_Point + [(Sr+Msib+2*Dbs) -Mcw/2], 0, Dm-Msib, Msib, Msih, tw, Chamber_Opacity, Path_Opacity)
    create_main_path(Map_Start_Point + [(Sr+Msib+2*Dbs+Dm) -Mcw/2], 0, Dm-Msib, Msib, Msih, tw, Chamber_Opacity, Path_Opacity)
    create_main_path(Map_Start_Point + [(Sr+Msib+2*Dbs+2*Dm) -Mcw/2], 0, Dbs-Msib, Msib, Msih, tw, Chamber_Opacity, Path_Opacity)
    create_main_path(Map_Start_Point + [(Sr+Msib+3*Dbs+2*Dm) -Mcw/2], 0, Dbs-Msib, Msib, Msih, tw, Chamber_Opacity, Path_Opacity)
    create_main_path(Map_Start_Point + [(Sr+Msib+4*Dbs+2*Dm) -Mcw/2], 0, Sr, Msib, Msih, tw, Chamber_Opacity, Path_Opacity)

    % Create junctions
    junction_path(Map_Start_Point + [Sr -Mcw/2], 0, Msib, Mcw, Msih, tw, Chamber_Opacity, Path_Opacity)
    junction_path(Map_Start_Point + [(Sr+Dbs) -Mcw/2], 0, Msib, Mcw, Msih, tw, Chamber_Opacity, Path_Opacity)
    junction_path(Map_Start_Point + [(Sr+2*Dbs) -Mcw/2], 0, Msib, Mcw, Msih, tw, Chamber_Opacity, Path_Opacity)
    junction_path(Map_Start_Point + [(Sr+2*Dbs+Dm) -Mcw/2], 0, Msib, Mcw, Msih, tw, Chamber_Opacity, Path_Opacity)
    junction_path(Map_Start_Point + [(Sr+2*Dbs+2*Dm) -Mcw/2], 0, Msib, Mcw, Msih, tw, Chamber_Opacity, Path_Opacity)
    junction_path(Map_Start_Point + [(Sr+3*Dbs+2*Dm) -Mcw/2], 0, Msib, Mcw, Msih, tw, Chamber_Opacity, Path_Opacity)
    junction_path(Map_Start_Point + [(Sr+4*Dbs+2*Dm) -Mcw/2], 0, Msib, Mcw, Msih, tw, Chamber_Opacity, Path_Opacity)

    %payload area
    payload_area(Map_Start_Point + [(map_x_length/2-Psm/2) -Mcw/2-Msil-Psm], 0, Psm, Psm, Psh, Msib, Msih, tw, Chamber_Opacity, Path_Opacity)

    % Create the left most rail
    x = Map_Start_Point(1);
    y = Map_Start_Point(2)-Mcw/2;
    z = 0;

    [X, Y] = meshgrid([x, x+tw], [y, y+Mcw]);
    Z_bottom = ones(size(X)) * (z + Msih-0.1);
    Z_top = ones(size(X)) * (z + Msih);
    h1 = mesh(X, Y, Z_bottom);
    h2 = mesh(X, Y, Z_top);

    % Apply properties to all mesh objects
    meshes_structure = [h1; h2];

    % Create the left most rail
    x = Map_Start_Point(1)+map_x_length;
    y = Map_Start_Point(2)-Mcw/2;
    z = 0;

    [X, Y] = meshgrid([x-tw, x], [y, y+Mcw]);
    Z_bottom = ones(size(X)) * (z + Msih-0.1);
    Z_top = ones(size(X)) * (z + Msih);
    h3 = mesh(X, Y, Z_bottom);
    h4 = mesh(X, Y, Z_top);

    % Apply properties to all mesh objects
    meshes_structure = [meshes_structure; h3; h4];

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

    % Set axis properties
    axis equal;
    axis([0 x_axis 0 y_axis 0 z_axis]);
    grid off;
    set(gca, 'XColor', 'none', 'YColor', 'none', 'ZColor', 'none');
    fig = gcf;
end

function [coords, tangents] = create_dome(x, y, radius, Msib, Msih, track_width, y_axis, Chamber_Opacity, Path_Opacity)
    [theta, phi] = meshgrid(linspace(0, 2*pi, 40), linspace(0, pi/2, 20));
    X = x + radius * cos(theta) .* cos(phi);
    Y = y + radius * sin(theta) .* cos(phi);
    Z =  radius * sin(phi);
    % Create dome surface with metallic appearance
    surf(X, Y, Z, ...
        'FaceColor', [1 1 1], ... %[0.85 0.85 0.85]
        'EdgeColor', 'none', ...
        'FaceAlpha', Chamber_Opacity, ...
        'FaceLighting', 'gouraud');

    % Dome rail track
    track_radius = sqrt(radius^2 - Msih^2);
    theta = linspace(0, 2*pi, 160);
    subtend_angle = atan2(Msib,2*track_radius);

    % Determine which portion to remove based on dome position
    if y < (y_axis/2)
        theta_range = (theta > pi/2 + pi/50 - subtend_angle) & (theta < pi/2 - pi/50 + subtend_angle);
    else
        theta_range = (theta > 3*pi/2 + pi/50 - subtend_angle) & (theta < 3*pi/2 - pi/50 + subtend_angle);
    end

    % Split theta into segments
    theta_segments = {};
    current_segment = [];
    for i = 1:length(theta)
        if ~theta_range(i)
            current_segment = [current_segment, theta(i)];
        else
            if ~isempty(current_segment)
                theta_segments{end+1} = current_segment;
                current_segment = [];
            end
        end
    end
    if ~isempty(current_segment)
        theta_segments{end+1} = current_segment;
    end

    inner_radius = track_radius-track_width;
    outer_radius = track_radius;

    % Create and draw each segment separately
    for seg = 1:length(theta_segments)
        theta_seg = theta_segments{seg};

        % Create coordinates for bottom circle
        x_outer_bottom = x + outer_radius * cos(theta_seg);
        y_outer_bottom = y + outer_radius * sin(theta_seg);
        z_outer_bottom = ones(size(theta_seg)) * (Msih - 0.1);

        x_inner_bottom = x + inner_radius * cos(theta_seg);
        y_inner_bottom = y + inner_radius * sin(theta_seg);
        z_inner_bottom = ones(size(theta_seg)) * (Msih - 0.1);

        % Create coordinates for top circle
        x_outer_top = x + outer_radius * cos(theta_seg);
        y_outer_top = y + outer_radius * sin(theta_seg);
        z_outer_top = ones(size(theta_seg)) * Msih;

        x_inner_top = x + inner_radius * cos(theta_seg);
        y_inner_top = y + inner_radius * sin(theta_seg);
        z_inner_top = ones(size(theta_seg)) * Msih;

        % Create top and bottom faces for this segment
        patch([x_outer_bottom, fliplr(x_inner_bottom)], ...
            [y_outer_bottom, fliplr(y_inner_bottom)], ...
            [z_outer_bottom, fliplr(z_inner_bottom)], ...
            [1 1 1], 'FaceAlpha', Path_Opacity, 'EdgeColor', 'none', 'FaceLighting', 'gouraud');

        patch([x_outer_top, fliplr(x_inner_top)], ...
            [y_outer_top, fliplr(y_inner_top)], ...
            [z_outer_top, fliplr(z_inner_top)], ...
            [1 1 1], 'FaceAlpha', Path_Opacity, 'EdgeColor', 'none', 'FaceLighting', 'gouraud');

        % Create walls for this segment
        for i = 1:length(theta_seg)-1
            % Outer wall
            patch([x_outer_bottom(i), x_outer_bottom(i+1), x_outer_top(i+1), x_outer_top(i)], ...
                [y_outer_bottom(i), y_outer_bottom(i+1), y_outer_top(i+1), y_outer_top(i)], ...
                [z_outer_bottom(i), z_outer_bottom(i+1), z_outer_top(i+1), z_outer_top(i)], ...
                [1 1 1], 'FaceAlpha', Path_Opacity, 'EdgeColor', 'none', 'FaceLighting', 'gouraud');

            % Inner wall
            patch([x_inner_bottom(i), x_inner_bottom(i+1), x_inner_top(i+1), x_inner_top(i)], ...
                [y_inner_bottom(i), y_inner_bottom(i+1), y_inner_top(i+1), y_inner_top(i)], ...
                [z_inner_bottom(i), z_inner_bottom(i+1), z_inner_top(i+1), z_inner_top(i)], ...
                [1 1 1], 'FaceAlpha', Path_Opacity, 'EdgeColor', 'none', 'FaceLighting', 'gouraud');
        end
    end


    % Calculate number of points needed for 0.25m spacing
    arc_length = (track_radius-track_width/2) * (2*pi - 2*subtend_angle);
    num_points = 2*ceil(ceil(arc_length / 0.1)/2);

    % Calculate arc length
    if y < (y_axis/2)
        % Bottom dome: gap at pi/2
        theta = [];
        theta = [theta,linspace(pi/2+subtend_angle, 3*pi/2, 0.5*num_points)];
        theta = [theta,linspace(-pi/2, pi/2-subtend_angle, 0.5*num_points)];
        theta = theta(:, end:-1:1);
    else
        % Top dome: gap at 3pi/2
        theta = linspace(-pi/2+subtend_angle, 3*pi/2-subtend_angle, num_points);
    end
    
    % Generate path coordinates
    x_coords = x + (track_radius-track_width/2) * cos(theta);
    y_coords = y + (track_radius-track_width/2) * sin(theta);
    z_coords = ones(size(theta)) * (Msih-0.1);
    
    % Combine coordinates
    coords = [x_coords', y_coords', z_coords'];

    % Tangent vectors (-sin(θ), cos(θ), 0) for clockwise motion
    arc_tangents = [-sin(theta)', cos(theta)', zeros(size(theta))'];
    
    % Normalize tangent vectors
    magnitudes = sqrt(sum(arc_tangents.^2, 2));
    tangents = arc_tangents ./ magnitudes;
end

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

function [coords, tangents] = create_main_path(Cube_corner, base_z, len, breadth, height, track_width, Chamber_Opacity, Path_Opacity)
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
    
    [X, Z] = meshgrid([x, x+len], [z, z+height]);
    Y_front = ones(size(X)) * y;
    Y_back = ones(size(X)) * (y + breadth);
    h3 = mesh(X, Y_front, Z);
    h4 = mesh(X, Y_back, Z);

    % Create the top rail
    [X, Y] = meshgrid([x, x+len], [y, y+track_width]);
    Z_bottom = ones(size(X)) * (z + height-0.1);
    Z_top = ones(size(X)) * (z + height);
    h5 = mesh(X, Y, Z_bottom);
    h6 = mesh(X, Y, Z_top);

    x_points = x+track_width/2:0.1:(x+len-track_width/2);
    coords.top = [x_points', repmat(y+track_width/2, length(x_points), 1), repmat((z + height-0.1), length(x_points), 1)];
    tangents.top = repmat([1, 0, 0], length(x_points), 1); 

    % Create the bottom rail
    [X, Y] = meshgrid([x, x+len], [y+breadth-track_width, y+breadth]);
    Z_bottom = ones(size(X)) * (z + height-0.1);
    Z_top = ones(size(X)) * (z + height);
    h7 = mesh(X, Y, Z_bottom);
    h8 = mesh(X, Y, Z_top);

    x_points = x+track_width/2:0.1:(x+len-track_width/2);
    coords.bottom = [x_points', repmat(y+breadth-track_width/2, length(x_points), 1), repmat((z + height-0.1), length(x_points), 1)];
    tangents.bottom = repmat([1, 0, 0], length(x_points), 1); 

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

function [coords, tangents] = junction_path(Cube_corner, base_z, len, breadth, height, track_width, Chamber_Opacity, Path_Opacity)
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

    % Create the bottom rail
    [X, Y] = meshgrid([x, x+len], [y, y+track_width]);
    Z_bottom = ones(size(X)) * (z + height-0.1);
    Z_top = ones(size(X)) * (z + height);
    h3 = mesh(X, Y, Z_bottom);
    h4 = mesh(X, Y, Z_top);
    
    x_points = x+track_width/2:0.1:(x+len-track_width/2);
    coords.bottom = [x_points', repmat(y+track_width/2, length(x_points), 1), repmat((z + height-0.1), length(x_points), 1)];
    tangents.bottom = repmat([1, 0, 0], length(x_points), 1);  % Points right

    % Create the top rail
    [X, Y] = meshgrid([x, x+len], [y+breadth-track_width, y+breadth]);
    Z_bottom = ones(size(X)) * (z + height-0.1);
    Z_top = ones(size(X)) * (z + height);
    h5 = mesh(X, Y, Z_bottom);
    h6 = mesh(X, Y, Z_top);

    coords.top = [x_points', repmat(y+breadth-track_width/2, length(x_points), 1), repmat((z + height-0.1), length(x_points), 1)];
    tangents.top = repmat([1, 0, 0], length(x_points), 1);     % Points right

    % Create the right rail
    [X, Y] = meshgrid([x+breadth-track_width, x+breadth], [y, y+breadth]);
    Z_bottom = ones(size(X)) * (z + height-0.1);
    Z_top = ones(size(X)) * (z + height);
    h7 = mesh(X, Y, Z_bottom);
    h8 = mesh(X, Y, Z_top);

    y_points = y+track_width/2:0.1:y+breadth-track_width/2;
    coords.right = [repmat(x+len-track_width/2, length(y_points), 1), y_points', repmat((z + height-0.1), length(y_points), 1)];
    tangents.right = repmat([0, 1, 0], length(y_points), 1);   % Points up

    % Create the left rail
    [X, Y] = meshgrid([x, x+track_width], [y, y+breadth]);
    Z_bottom = ones(size(X)) * (z + height-0.1);
    Z_top = ones(size(X)) * (z + height);
    h9 = mesh(X, Y, Z_bottom);
    h10 = mesh(X, Y, Z_top);

    coords.left = [repmat(x+track_width/2, length(y_points), 1), y_points', repmat((z + height-0.1), length(y_points), 1)];
    tangents.left = repmat([0, 1, 0], length(y_points), 1);   % Points up

    % Apply properties to all mesh objects
    meshes_structure = [h1; h2];

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

    meshes_rail = [h3; h4; h5; h6; h7; h8; h9; h10];

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

    coords = [coords3; coords4; coords1; coords5; coords2];
    tangents = [tangents3; tangents4; tangents1; tangents5; tangents2];

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