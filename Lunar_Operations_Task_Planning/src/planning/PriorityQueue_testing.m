addpath('C:\Users\ADHARSH\Desktop\Job_Search\Github\Robotics-Portfolio\Lunar_Operations_Task_Planning\src\environment\map_generation')

% [fig, paths] = create_3D_lunar_base();
% [fig, paths_branchless] = create_3D_lunar_base_branch_less();
start_node = 'SA01_1';
end_node = 'SA06_1';
tic 
final_path_array = Constrained_Rail_Astar_Array(paths, start_node, end_node);
disp(time);

tic
final_path_priority = Constrained_Rail_Astar_PriotiyQueue(paths_branchless, start_node, end_node);
time =  toc;
disp(time)