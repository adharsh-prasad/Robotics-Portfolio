addpath('C:\Users\ADHARSH\Desktop\Job_Search\Github\Robotics-Portfolio\Lunar_Operations_Task_Planning\src\environment\map_generation')

% % Create the 3D lunar base map
% [fig, paths] = create_3D_lunar_base();
% 
% % Benchmarking parameters
% num_pairs = 50;  % Number of start-end pairs
% num_runs = 10;   % Number of runs per pair
% resolutions = [0.1, 1];  % Test both resolutions for built-in, only 0.1 for custom
% 
% % Generate random start-end pairs
% node_list = keys(paths);
% node_array = cellstr(node_list);
% pairs = cell(num_pairs, 2);
% for i = 1:num_pairs
%     pairs{i,1} = node_array{randi(length(node_array))};
%     pairs{i,2} = node_array{randi(length(node_array))};
% end
% 
% % Initialize result arrays
% custom_times_real = zeros(num_pairs, 1);
% custom_times_cpu = zeros(num_pairs, 1);
% builtin_times_real = zeros(num_pairs, length(resolutions));
% builtin_times_cpu = zeros(num_pairs, length(resolutions));
% 
% % Benchmarking for custom A* (only for resolution 0.1)
% resolution = 0.1;
% [binary_map, ~] = create_binary_map(paths, fig, resolution);
% 
% for i = 1:num_pairs
%     start = pairs{i,1};
%     end_loc = pairs{i,2};
% 
%     tic_real = tic;
%     cpu_time = cputime;
%     for j = 1:num_runs
%         Constrained_Rail_Astar_Array(paths, start, end_loc);
%     end
%     custom_times_real(i) = toc(tic_real) / num_runs;
%     custom_times_cpu(i) = (cputime - cpu_time) / num_runs;
% end
% 
% % Benchmarking for built-in A* (for both resolutions)
% for res_idx = 1:length(resolutions)
%     resolution = resolutions(res_idx);
% 
%     [binary_map, ~] = create_binary_map(paths, fig, resolution);
%     map = binaryOccupancyMap(binary_map, resolution);
%     gridPlanner = plannerAStarGrid(map);
% 
%     for i = 1:num_pairs
%         start = pairs{i,1};
%         end_loc = pairs{i,2};
% 
%         start_coords = round(paths(start).coordinates(1, 1:2) / resolution);
%         end_coords = round(paths(end_loc).coordinates(1, 1:2) / resolution);
%         start_coords = [start_coords(2) start_coords(1)];
%         end_coords = [end_coords(2) end_coords(1)];
% 
%         tic_real = tic;
%         cpu_time = cputime;
%         for j = 1:num_runs
%             plan(gridPlanner, start_coords, end_coords);
%         end
%         builtin_times_real(i, res_idx) = toc(tic_real) / num_runs;
%         builtin_times_cpu(i, res_idx) = (cputime - cpu_time) / num_runs;
%     end
% end

% Calculate average times
avg_custom_time_real = mean(custom_times_real);
avg_custom_time_cpu = mean(custom_times_cpu);
avg_builtin_time_real = mean(builtin_times_real);
avg_builtin_time_cpu = mean(builtin_times_cpu);

% Create a 2x2 subplot figure
figure('Position', [0, 0, 1920, 1080]);

% Real-time comparison for resolution 0.1
subplot(2,2,1);
boxplot([custom_times_real, builtin_times_real(:,1)], 'Labels', {'Custom 0.1', 'Built-in 0.1'});
title('Real Time Comparison', 'FontSize', 14);
ylabel('Time (seconds)', 'FontSize', 12);
% result_text = sprintf('Custom A*: %.6f s\nBuilt-in A*: %.6f s', avg_custom_time_real, avg_builtin_time_real(1));
% legend(result_text, 'Location', 'northoutside', 'FontSize', 10);

% Real-time comparison for resolution 1
subplot(2,2,2);
boxplot([custom_times_real, builtin_times_real(:,2)], 'Labels', {'Custom 0.1', 'Built-in 1'});
title('Real Time Comparison', 'FontSize', 14);
ylabel('Time (seconds)', 'FontSize', 12);
% result_text = sprintf('Custom A*: %.6f s\nBuilt-in A*: %.6f s', avg_custom_time_real, avg_builtin_time_real(2));
% legend(result_text, 'Location', 'northoutside', 'FontSize', 10);

% CPU time comparison for resolution 0.1
subplot(2,2,3);
boxplot([custom_times_cpu, builtin_times_cpu(:,1)], 'Labels', {'Custom 0.1', 'Built-in 0.1'});
title('CPU Time Comparison', 'FontSize', 14);
ylabel('Time (seconds)', 'FontSize', 12);
% result_text = sprintf('Custom A*: %.6f s\nBuilt-in A*: %.6f s', avg_custom_time_cpu, avg_builtin_time_cpu(1));
% legend(result_text, 'Location', 'northoutside', 'FontSize', 10);

% CPU time comparison for resolution 1
subplot(2,2,4);
boxplot([custom_times_cpu, builtin_times_cpu(:,2)], 'Labels', {'Custom 0.1', 'Built-in 1'});
title('CPU Time Comparison', 'FontSize', 14);
ylabel('Time (seconds)', 'FontSize', 12);
% result_text = sprintf('Custom A*: %.6f s\nBuilt-in A*: %.6f s', avg_custom_time_cpu, avg_builtin_time_cpu(2));
% legend(result_text, 'Location', 'northoutside', 'FontSize', 10);

% Adjust the layout
sgtitle('A* Algorithm Performance Comparison', 'FontSize', 16);
