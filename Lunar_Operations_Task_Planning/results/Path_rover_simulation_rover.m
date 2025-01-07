% Create the 3D lunar base map
[fig, paths] = create_3D_lunar_base();
view(45, 30)

% Set up video writer
v = VideoWriter('lunar_rover_simulation.mp4', 'MPEG-4');
v.FrameRate = 60;  % Adjust frame rate as needed
v.Quality = 100;   % Maximum quality
open(v);

start_segment = 'SA01_1';
coords = paths(start_segment).coordinates;
tangents = paths(start_segment).tangents;
idx = randi(size(coords, 1));
start_loc = coords(idx, :);
start_path = coords(idx:-1:1,:);
start_tangents = tangents(idx:-1:1,:);

end_segment = 'SA06_1';
coords = paths(end_segment).coordinates;
tangents = paths(end_segment).tangents;
idx = randi(size(coords, 1));
end_loc = coords(idx, :);
end_path = coords(1:idx,:);
end_tangents = tangents(1:idx,:);

original_path = Constrained_Rail_Astar_Array(paths, start_segment, end_segment);


detailed_path = start_path;
tangents_path = start_tangents;

for i = 2:length(original_path)-1
    segment = original_path{i};
    coords = paths(segment).coordinates;
    tangents = paths(segment).tangents;

    % For middle segments, check orientation with next segment
    next_segment = original_path{i+1};
    next_start = paths(next_segment).coordinates(1,:);

    if norm(coords(end,:) - next_start) < norm(coords(1,:) - next_start)
        detailed_path = [detailed_path; coords];
        tangents_path = [ tangents_path;tangents];
    else
        detailed_path = [detailed_path; coords(end:-1:1,:)];
        tangents_path = [ tangents_path;tangents(end:-1:1,:)];
    end
end

detailed_path = [detailed_path; end_path];
tangents_path = [ tangents_path;end_tangents];

% Create base rover
base_rover = create_rover();

% Initialize rover plot
rover_plot = patch('Vertices', base_rover.Vertices, 'Faces', base_rover.Faces, 'FaceColor', 'red');

% Simulation loop with video capture
for i = 1:size(detailed_path, 1)
    current_position = detailed_path(i, :);
    current_tangent = tangents_path(i, :);
    
    % Rotate and position the rover
    rotated_vertices = rotate_cuboid(base_rover.Vertices, current_tangent, current_position);
    
    % Update the plot
    set(rover_plot, 'Vertices', rotated_vertices);
    drawnow;
    
    % Capture the frame
    frame = getframe(gcf);
    writeVideo(v, frame);
end

% Close the video file
close(v);

function rotated_vertices = rotate_cuboid(vertices, tangent, position)
    % Normalize the tangent vector
    t = tangent / norm(tangent);
    
    % Create a rotation matrix for x-y plane
    angle = atan2(t(2), t(1));
    R = [cos(angle), -sin(angle), 0;
         sin(angle), cos(angle), 0;
         0, 0, 1];
    
    % Rotate the vertices
    rotated_vertices = (R * vertices')';
    
    % Translate to the new position
    rotated_vertices = rotated_vertices + position;
end


function positioned_rover = position_rover(rover, target_position)
    positioned_rover = rover;
    positioned_rover.Vertices = positioned_rover.Vertices + target_position;
end
