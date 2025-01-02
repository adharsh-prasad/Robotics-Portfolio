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