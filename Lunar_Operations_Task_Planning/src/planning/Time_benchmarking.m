

addpath('C:\Users\ADHARSH\Desktop\Job_Search\Github\Robotics-Portfolio\Lunar_Operations_Task_Planning\src\environment\map_generation')
[fig,paths] = create_3D_lunar_base();

% Create and adjust binary map
binary_map = create_binary_map(paths, fig, 1);
binary_map = fliplr(binary_map);  % Correct orientation

% Convert path coordinates to grid coordinates
start_coords = paths("SA01_1").coordinates(50,1:2);
goal_coords = paths("SA06_2").coordinates(50,1:2);

% Scale coordinates to match grid
start = round([start_coords(1), start_coords(2)]);
goal = round([goal_coords(1), goal_coords(2)]);

% Create binary occupancy map from our grid
map = binaryOccupancyMap(binary_map);

% Create state space for 2D navigation
ss = stateSpaceSE2;
ss.StateBounds = [map.XWorldLimits; map.YWorldLimits; [-pi pi]];

% Create state validator
sv = validatorOccupancyMap(ss);
sv.Map = map;
sv.ValidationDistance = 0.1;

% Create A* planner
planner = plannerAStarGrid(map);

% Plan path
tic;
path = plan(planner, ceil(start), ceil(goal));
planning_time = toc;
disp('planning_time')
disp(planning_time)
% Visualize
binary_map = fliplr(binary_map);
    ax1 = fig.CurrentAxes;
imagesc(ax1, binary_map');
hold on
scatter3(path(:,1), path(:,2), ones(size(path(:,2))*2), 'r-', 'LineWidth', 2)
scatter3(start(1), start(2), ones(size(path(:,2))*2), 100, 'g', 'filled')
scatter3(goal(1), goal(2), ones(size(path(:,2))*2), 100, 'r', 'filled')



% node_list = keys(paths);
% node_to_idx = containers.Map(node_list, 1:length(node_list));
% 
% tic;
% final_path = Constrained_Rail_Astar_Array(paths, 'SA01_1', 'SA06_1');
% disp(toc)
% 
% for i = 1:size(final_path,2)
%     coords = paths(final_path(i)).coordinates;
%     scatter3(coords(:,1), coords(:,2), coords(:,3),'o','filled','MarkerFaceColor','red')
% end
% m = 1000;   % Number of test pairs
% n = length(node_list);
% pairs = zeros(m, 2);
% for i = 1:m
%     pairs(i,:) = datasample(1:n, 2, 'Replace', false);
% end
% 
% % Convert keylist to cell array once
% node_array = cellstr(node_list);
% 
% % Warm-up runs
% for i = 1:5
%     start = node_array{pairs(1,1)};
%     end_loc = node_array{pairs(1,2)};
%     Constrained_Rail_Astar_Array(paths, start, end_loc);
% end
% 
% % Measure total execution time
% tic
% for i = 1:m
%     start = node_array{pairs(i,1)};
%     end_loc = node_array{pairs(i,2)};
%     Constrained_Rail_Astar_Array(paths, start, end_loc);
% end
% total_time = toc;
% 
% % Calculate average time per path
% avg_time = total_time/m;
