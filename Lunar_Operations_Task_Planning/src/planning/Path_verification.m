node_list = keys(paths);
    node_to_idx = containers.Map(node_list, 1:length(node_list));
tic;
Constrained_Rail_Astar_Array(paths, 'SA01_1', 'SA06_2', node_list, node_to_idx)
disp(toc)

clc
m = 1000;   % Number of test pairs
pairs = zeros(m, 2);
for i = 1:m
    pairs(i,:) = datasample(1:n, 2, 'Replace', false);
end

% Convert keylist to cell array once
key_array = cellstr(keylist);

% Warm-up runs
for i = 1:5
    start = key_array{pairs(1,1)};
    end_loc = key_array{pairs(1,2)};
    Constrained_Rail_Astar_Array(paths, start, end_loc, node_list, node_to_idx);
end

% Measure total execution time
tic
for i = 1:m
    start = key_array{pairs(i,1)};
    end_loc = key_array{pairs(i,2)};
    Constrained_Rail_Astar_Array(paths, start, end_loc, node_list, node_to_idx);
end
total_time = toc;

% Calculate average time per path
avg_time = total_time/m;
