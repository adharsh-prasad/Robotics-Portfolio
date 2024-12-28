    % function create_3D_lunar_base()
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
    Esm = 4;%Empty space dimensions
    Dm = Esm/2 + Sr + So;%Distance_in_middle
    tw = 0.3; %Track width

    %Axis dimensions
    map_x_length = 2*(Sr + 2*Dbs + 1*Dm + Msib/2);
    map_y_length = (Sr*4 + Msil*2 + Mcw);
    x_axis = 5*(floor(map_x_length/5) + 1);
    y_axis = 5*(floor(map_y_length/5) + 1);
    z_axis = 5*(floor(Sr/5) + 1);
    map_x_axis_offset = (x_axis - map_x_length)/2;
    map_y_axis_offset = (y_axis - map_y_length)/2;

    % Create fullscreen figure with black background
    fig = figure('Units', 'normalized', 'Position', [0 0 1 1], ...
                'Color', 'black', 'MenuBar', 'none', ...
                'ToolBar', 'none', 'WindowState', 'fullscreen');
    ax = axes('Color', 'black', 'Position', [0 0 1 1]);
    hold on;
    
    % Set view for top-down perspective with slight angle
    view(0, 90);
    % view(270, 0);
    % view(45, 30);
    lighting gouraud;
    camlight('headlight');
    
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
        create_dome(Superadobe_centers(i,1), Superadobe_centers(i,2), Sr, Msib, Msih, tw, y_axis);
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

    % Create intersections
    for i = 1:size(Intersection_centers, 1)
        create_connecting_cube(Intersection_centers(i,:), 0, Msib, Msil, Msih)
    end

    % Create the main path
    create_connecting_cube(Map_Start_Point - [0 Mcw/2], 0, map_x_length, Msib, Msih)

    % Set axis properties
    axis equal;
    axis([0 x_axis 0 y_axis 0 z_axis]);
    grid off;
    set(gca, 'XColor', 'none', 'YColor', 'none', 'ZColor', 'none');
    % end

function create_dome(x, y, radius, Msib, Msih, track_width, y_axis)
    [theta, phi] = meshgrid(linspace(0, 2*pi, 40), linspace(0, pi/2, 20));
    X = x + radius * cos(theta) .* cos(phi);
    Y = y + radius * sin(theta) .* cos(phi);
    Z =  radius * sin(phi);
    % Create dome surface with metallic appearance
    surf(X, Y, Z, ...
        'FaceColor', [1 1 1], ... %[0.85 0.85 0.85]
        'EdgeColor', 'none', ...
        'FaceAlpha', 0.3, ...
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
            [1 1 1], 'FaceAlpha', 0.3, 'EdgeColor', 'none', 'FaceLighting', 'gouraud');

        patch([x_outer_top, fliplr(x_inner_top)], ...
            [y_outer_top, fliplr(y_inner_top)], ...
            [z_outer_top, fliplr(z_inner_top)], ...
            [1 1 1], 'FaceAlpha', 0.3, 'EdgeColor', 'none', 'FaceLighting', 'gouraud');

        % Create walls for this segment
        for i = 1:length(theta_seg)-1
            % Outer wall
            patch([x_outer_bottom(i), x_outer_bottom(i+1), x_outer_top(i+1), x_outer_top(i)], ...
                [y_outer_bottom(i), y_outer_bottom(i+1), y_outer_top(i+1), y_outer_top(i)], ...
                [z_outer_bottom(i), z_outer_bottom(i+1), z_outer_top(i+1), z_outer_top(i)], ...
                [1 1 1], 'FaceAlpha', 0.3, 'EdgeColor', 'none', 'FaceLighting', 'gouraud');

            % Inner wall
            patch([x_inner_bottom(i), x_inner_bottom(i+1), x_inner_top(i+1), x_inner_top(i)], ...
                [y_inner_bottom(i), y_inner_bottom(i+1), y_inner_top(i+1), y_inner_top(i)], ...
                [z_inner_bottom(i), z_inner_bottom(i+1), z_inner_top(i+1), z_inner_top(i)], ...
                [1 1 1], 'FaceAlpha', 0.3, 'EdgeColor', 'none', 'FaceLighting', 'gouraud');
        end
    end

end

function create_connecting_cube(Cube_corner, base_z, length, breadth, height)
    x = Cube_corner(1);
    y = Cube_corner(2);
    z = base_z;

    % Define the grid for the mesh
    [X, Y] = meshgrid([x, x+length], [y, y+breadth]);
    Z_bottom = ones(size(X)) * z;
    Z_top = ones(size(X)) * (z + height);

    % Create surfaces with matching properties
    h1 = mesh(X, Y, Z_bottom);
    h2 = mesh(X, Y, Z_top);
    
    [Y, Z] = meshgrid([y, y+breadth], [z, z+height]);
    X_front = ones(size(Y)) * x;
    X_back = ones(size(Y)) * (x + length);
    h3 = mesh(X_front, Y, Z);
    h4 = mesh(X_back, Y, Z);

    % Create the left top rail
    [X, Y] = meshgrid([x, x+0.3], [y, y+breadth]);
    Z_bottom = ones(size(X)) * (z + height-0.1);
    Z_top = ones(size(X)) * (z + height);
    h5 = mesh(X, Y, Z_bottom);
    h6 = mesh(X, Y, Z_top);

    % Create the right top rail
    [X, Y] = meshgrid([x+length-0.3, x+length], [y, y+breadth]);
    Z_bottom = ones(size(X)) * (z + height-0.1);
    Z_top = ones(size(X)) * (z + height);
    h7 = mesh(X, Y, Z_bottom);
    h8 = mesh(X, Y, Z_top);

    % Apply properties to all mesh objects
    meshes = [h1; h2; h3; h4; h5; h6; h7; h8];
    for h = meshes'
        set(h, 'FaceColor', [1 1 1], ...
               'EdgeColor', 'none', ...
               'FaceAlpha', 0.3, ...
               'FaceLighting', 'gouraud', ...
               'AmbientStrength', 0.3, ...
               'DiffuseStrength', 0.8, ...
               'SpecularStrength', 0.9, ...
               'SpecularExponent', 25);
    end
end