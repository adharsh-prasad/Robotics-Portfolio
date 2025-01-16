addpath('C:\Users\ADHARSH\Desktop\Job_Search\Github\Robotics-Portfolio\Lunar_Operations_Task_Planning\src\environment\map_generation')
% [fig, paths] = create_3D_lunar_base_branch_less();

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

start_node = 'SA01_1';
end_node = 'SA06_1';

time = 0;
for i = 1:1000
tic
path = Custom_Rail_Astar(id_paths, paths(start_node).id, paths(end_node).id);
temp = toc;
time = time + temp;
end

final = '';
for i = 1:length(path)
disp(id_to_name(path(i)))
end
