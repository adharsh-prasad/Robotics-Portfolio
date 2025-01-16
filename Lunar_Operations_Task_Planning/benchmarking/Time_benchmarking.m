addpath('C:\Users\ADHARSH\Desktop\Job_Search\Github\Robotics-Portfolio\Lunar_Operations_Task_Planning\src\environment\map_generation')
addpath('C:\Users\ADHARSH\Desktop\Job_Search\Github\Robotics-Portfolio\Lunar_Operations_Task_Planning\src\planning')

% Create the 3D lunar base map
[fig, paths] = create_3D_lunar_base();
node_list = keys(paths);
for i = 1:length(node_list)
    paths(node_list{i}).id = i;
end

% Convert string connections to ID connections
segment = keys(paths);
for j = 1:length(keys(paths))
    current_connections = paths(segment{j}).connections;
    id_connections = zeros(length(current_connections), 1);
    for i = 1:length(current_connections)
        id_connections(i) = paths(current_connections{i}).id;
    end
    paths(segment{j}).connections_id = id_connections;
    paths(segment{j}).mid_point = (paths(segment{j}).start_point + paths(segment{j}).end_point)/2;
end

node_list = keys(paths);
for i = 1:length(node_list)
    paths(node_list{i}).id = i;
end

% Convert string connections to ID connections
segment = keys(paths);
for j = 1:length(keys(paths))
    current_connections = paths(segment{j}).connections;
    id_connections = zeros(length(current_connections), 1);
    for i = 1:length(current_connections)
        id_connections(i) = paths(current_connections{i}).id;
    end
    paths(segment{j}).connections_id = id_connections;
    paths(segment{j}).mid_point = (paths(segment{j}).start_point + paths(segment{j}).end_point)/2;
end

% Create ID-based path structure
id_paths = struct();

% First create the complete mappings
node_list = keys(paths);
name_to_id = containers.Map('KeyType', 'char', 'ValueType', 'double');
id_to_name = containers.Map('KeyType', 'double', 'ValueType', 'char');

% Create all mappings first
for i = 1:length(node_list)
    segment_name = node_list{i};
    name_to_id(segment_name) = i;
    id_to_name(i) = segment_name;
end

% Populate the ID-based structure
for i = 1:length(node_list)
    segment_name = node_list{i};

    % Create mappings
    name_to_id(segment_name) = i;
    id_to_name(i) = segment_name;

    % Get original path data
    path_data = paths(segment_name);

    % Convert string connections to IDs
    id_connections = zeros(length(path_data.connections), 1);
    for j = 1:length(path_data.connections)
        id_connections(j) = name_to_id(path_data.connections{j});
    end

    % Create ID-based path structure
    id_paths(i).connections = id_connections;
    id_paths(i).coordinates = path_data.coordinates;
    id_paths(i).tangents = path_data.tangents;
    id_paths(i).start_point = path_data.start_point;
    id_paths(i).end_point = path_data.end_point;
    id_paths(i).distance = path_data.distance;
    id_paths(i).name = segment_name;  % Keep original name for reference
    id_paths(i).mid_point = (id_paths(i).start_point + id_paths(i).end_point)/2;
end

% Benchmarking parameters
num_pairs = 50;  % Number of start-end pairs
num_runs = 10;   % Number of runs per pair
resolutions = [0.1, 1];  % Test both resolutions for built-in, only 0.1 for custom

% Generate random start-end pairs
node_list = keys(paths);
node_array = cellstr(node_list);
pairs = cell(num_pairs, 2);
for i = 1:num_pairs
    pairs{i,1} = node_array{randi(length(node_array))};
    pairs{i,2} = node_array{randi(length(node_array))};
end

% Initialize result arrays for all algorithms
custom_times_real = zeros(num_pairs, 1);     % Custom A*
custom_times_cpu = zeros(num_pairs, 1);     % Custom A*
priority_times_real = zeros(num_pairs, 1);    % Priority Queue A*
priority_times_cpu = zeros(num_pairs, 1);      % Priority Queue A* cpu
builtin_times_real = zeros(num_pairs, 2);    % Built-in A* (2 resolutions)
builtin_times_cpu = zeros(num_pairs, 2);


% Benchmarking for Custom A*
for i = 1:num_pairs
    start = pairs{i,1};
    end_loc = pairs{i,2};

    tic_real = tic;
    cpu_time = cputime;
    for j = 1:num_runs
        Custom_Rail_Astar(id_paths, paths(start).id, paths(end_loc).id);
    end
    custom_times_real(i) = toc(tic_real) / num_runs;
    custom_times_cpu(i) = (cputime - cpu_time) / num_runs;
end

% Benchmarking for priority queue A* (only for resolution 0.1)
resolution = 0.1;
[binary_map, ~] = create_binary_map(paths, fig, resolution);

for i = 1:num_pairs
    start = pairs{i,1};
    end_loc = pairs{i,2};

    tic_real = tic;
    cpu_time = cputime;
    for j = 1:num_runs
        Constrained_Rail_Astar_PriotiyQueue(paths, start, end_loc);
    end
    priority_times_real(i) = toc(tic_real) / num_runs;
    priority_times_cpu(i) = (cputime - cpu_time) / num_runs;
end

% Benchmarking for built-in A* (for both resolutions)
for res_idx = 1:length(resolutions)
    resolution = resolutions(res_idx);

    [binary_map, ~] = create_binary_map(paths, fig, resolution);
    map = binaryOccupancyMap(binary_map, 1/resolution);
    gridPlanner = plannerAStarGrid(map);

    for i = 1:num_pairs
        start = pairs{i,1};
        end_loc = pairs{i,2};

        start_coords = round(paths(start).coordinates(1, 1:2) / resolution);
        end_coords = round(paths(end_loc).coordinates(1, 1:2) / resolution);
        start_coords = [start_coords(2) start_coords(1)];
        end_coords = [end_coords(2) end_coords(1)];

        tic_real = tic;
        cpu_time = cputime;
        for j = 1:num_runs
            plan(gridPlanner, start_coords, end_coords);
        end
        builtin_times_real(i, res_idx) = toc(tic_real) / num_runs;
        builtin_times_cpu(i, res_idx) = (cputime - cpu_time) / num_runs;
    end
end

% Calculate average times
avg_custom_time_real = mean(priority_times_real);
avg_custom_time_cpu = mean(priority_times_cpu);
avg_builtin_time_real = mean(builtin_times_real);
avg_builtin_time_cpu = mean(builtin_times_cpu);

% Create figure with black background
fig = figure('Units', 'normalized', 'Position', [0 0 1 1], ...
                'Color', 'black', 'MenuBar', 'none', ...
                'ToolBar', 'none', 'WindowState', 'fullscreen');
    ax = axes('Color', 'black', 'Position', [0 0 1 1]);
    hold on;

% Real-time comparison for all algorithms
subplot(2,2,1);
boxplot([custom_times_real, priority_times_real, builtin_times_real(:,1), builtin_times_real(:,2)], ...
    'Labels', {'Custom A*', 'Priority Queue A*', 'Built-in (0.1)', 'Built-in (1.0)'});
title('Real Time Comparison', 'FontSize', 14, 'Color', 'white');
ylabel('Time (seconds)', 'FontSize', 12, 'Color', 'white');
set(gca, 'Color', 'black', 'XColor', 'white', 'YColor', 'white');

% CPU time comparison for all algorithms
subplot(2,2,2);
boxplot([custom_times_cpu, priority_times_cpu, builtin_times_cpu(:,1), builtin_times_cpu(:,2)], ...
    'Labels', {'Custom A*', 'Priority Queue A*', 'Built-in (0.1)', 'Built-in (1.0)'});
title('CPU Time Comparison', 'FontSize', 14, 'Color', 'white');
ylabel('Time (seconds)', 'FontSize', 12, 'Color', 'white');
set(gca, 'Color', 'black', 'XColor', 'white', 'YColor', 'white');

% Add statistical analysis with mean values on bars
subplot(2,2,3);
avg_times = [mean(custom_times_real), mean(priority_times_real), ...
    mean(builtin_times_real(:,1)), mean(builtin_times_real(:,2))];
std_times = [std(custom_times_real), std(priority_times_real), ...
    std(builtin_times_real(:,1)), std(builtin_times_real(:,2))];

bar_plot = bar(avg_times);
hold on;
errorbar(1:4, avg_times, std_times, 'k', 'LineStyle', 'none');

% Add text labels on top of each bar
for i = 1:length(avg_times)
    text(i, avg_times(i), sprintf('%.2f ms', avg_times(i)*1000), ...
        'HorizontalAlignment', 'center', ...
        'VerticalAlignment', 'bottom', ...
        'Color', 'white', ...
        'FontSize', 10);
end

set(gca, 'XTickLabel', {'Custom A*', 'Priority Queue A*', ...
    'Built-in (0.1)', 'Built-in (1.0)'}, ...
    'Color', 'black', 'XColor', 'white', 'YColor', 'white');
title('Average Execution Time Comparison', 'FontSize', 14, 'Color', 'white');
ylabel('Time (seconds)', 'FontSize', 12, 'Color', 'white');
xtickangle(45);


% Success rate comparison
subplot(2,2,4);
success_rate = [sum(~isnan(custom_times_real))/length(custom_times_real), ...
    sum(~isnan(priority_times_real))/length(priority_times_real), ...
    sum(~isnan(builtin_times_real(:,1)))/length(builtin_times_real), ...
    sum(~isnan(builtin_times_real(:,2)))/length(builtin_times_real)] * 100;
bar(success_rate, 'FaceColor', 'w');
set(gca, 'XTickLabel', {'Custom A*', 'Priority Queue A*', 'Built-in (0.1)', 'Built-in (1.0)'}, ...
    'Color', 'black', 'XColor', 'white', 'YColor', 'white');
title('Path Finding Success Rate', 'FontSize', 14, 'Color', 'white');
ylabel('Success Rate (%)', 'FontSize', 12, 'Color', 'white');
xtickangle(45);

% Overall title
sgtitle('A* Algorithm Performance Comparison', 'FontSize', 16, 'Color', 'white');
