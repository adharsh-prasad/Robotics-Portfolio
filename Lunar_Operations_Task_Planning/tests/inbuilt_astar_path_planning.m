addpath('C:\Users\ADHARSH\Desktop\Job_Search\Github\Robotics-Portfolio\Lunar_Operations_Task_Planning\src\environment\map_generation')
addpath('C:\Users\ADHARSH\Desktop\Job_Search\Github\Robotics-Portfolio\Lunar_Operations_Task_Planning\benchmarking')
[fig, paths] = create_3D_lunar_base();

resolution = 1;

[binary_map, ~] = create_binary_map(paths, fig, resolution);
map = binaryOccupancyMap(binary_map, 1/resolution);
gridPlanner = plannerAStarGrid(map);

start_coords = round(paths('SA01_1').coordinates(50, 1:2)/resolution);
end_coords = round(paths('SA06_1').coordinates(50, 1:2)/resolution);
start_coords = [start_coords(2) start_coords(1)];
end_coords = [end_coords(2) end_coords(1)];

z_height = 1.9;

visualize_binary_map(binary_map, z_height)
hold on

path = plan(gridPlanner, start_coords, end_coords);

% Plot path with slight offset above binary map
scatter3(path(:,2)*resolution, path(:,1)*resolution, ones(size(path,1),1)*(z_height+0.1), ...
    40, 'g', 'filled')

% Plot start and end points
scatter3(start_coords(2)*resolution, start_coords(1)*resolution, z_height+0.1, 100, 'g', 'filled')
scatter3(end_coords(2)*resolution, end_coords(1)*resolution, z_height+0.1, 100, 'r', 'filled')

% Keep axis properties
% axis equal
grid off