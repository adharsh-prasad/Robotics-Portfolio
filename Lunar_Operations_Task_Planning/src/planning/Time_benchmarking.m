addpath('C:\Users\ADHARSH\Desktop\Job_Search\Github\Robotics-Portfolio\Lunar_Operations_Task_Planning\src\environment\map_generation')
[fig,paths] = create_3D_lunar_base();

node_list = keys(paths);
node_to_idx = containers.Map(node_list, 1:length(node_list));

tic;
Constrained_Rail_Astar_Array(paths, 'SA01_1', 'SA05_1')
disp(toc)

m = 1000;   % Number of test pairs
n = length(node_list);
pairs = zeros(m, 2);
for i = 1:m
    pairs(i,:) = datasample(1:n, 2, 'Replace', false);
end

% Convert keylist to cell array once
node_array = cellstr(node_list);

% Warm-up runs
for i = 1:5
    start = node_array{pairs(1,1)};
    end_loc = node_array{pairs(1,2)};
    Constrained_Rail_Astar_Array(paths, start, end_loc);
end

% Measure total execution time
tic
for i = 1:m
    start = node_array{pairs(i,1)};
    end_loc = node_array{pairs(i,2)};
    Constrained_Rail_Astar_Array(paths, start, end_loc);
end
total_time = toc;

% Calculate average time per path
avg_time = total_time/m;
