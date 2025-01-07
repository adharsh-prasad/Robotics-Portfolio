% Create the 3D lunar base map
[fig, paths] = create_3D_lunar_base();
view(45, 30)
start_segment = 'SA01_1';
coords = paths(start_segment).coordinates;
idx = randi(size(coords, 1));
start_loc = coords(idx, :);
start_path = coords(idx:-1:1,:);

end_segment = 'SA06_1';
coords = paths(end_segment).coordinates;
idx = randi(size(coords, 1));
end_loc = coords(idx, :);
end_path = coords(1:idx,:);

original_path = Constrained_Rail_Astar_Array(paths, start_segment, end_segment);

detailed_path = start_path;

for i = 2:length(original_path)-1
    segment = original_path{i};
    coords = paths(segment).coordinates;

    % For middle segments, check orientation with next segment
    next_segment = original_path{i+1};
    next_start = paths(next_segment).coordinates(1,:);

    if norm(coords(end,:) - next_start) < norm(coords(1,:) - next_start)
        detailed_path = [detailed_path; coords];
    else
        detailed_path = [detailed_path; coords(end:-1:1,:)];
    end
end

detailed_path = [detailed_path; end_path];

% Colour gradient array for scatter plot
num_points = size(detailed_path, 1);
colors = [linspace(0, 1, num_points)', linspace(1, 0, num_points)', zeros(num_points, 1)];

% Set up video writer
v = VideoWriter('path_verification.mp4', 'MPEG-4');
v.FrameRate = 60; % Adjust as needed
open(v);

fig;

% Set up the plot
title('Path Verification');
xlabel('X');
ylabel('Y');
zlabel('Z');

% Plot start and end points
scatter3(detailed_path(1,1), detailed_path(1,2), detailed_path(1,3), 200, 'g', 'filled');
scatter3(detailed_path(end,1), detailed_path(end,2), detailed_path(end,3), 200, 'r', 'filled');

% Plot each point of the path
for i = 1:size(detailed_path, 1)
    scatter3(detailed_path(i,1), detailed_path(i,2), detailed_path(i,3), 40, colors(i,:), 'filled');
    drawnow;
    
    % Capture the frame
    frame = getframe(gcf);
    writeVideo(v, frame);
end

% Connect the points with a line
plot3(detailed_path(:,1), detailed_path(:,2), detailed_path(:,3), 'r-', 'LineWidth', 2);

% Capture final frame
frame = getframe(gcf);
writeVideo(v, frame);

% Close the video writer
close(v);

