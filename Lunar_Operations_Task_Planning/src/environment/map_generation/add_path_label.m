% addpath('C:\Users\ADHARSH\Desktop\Job_Search\Github\Robotics-Portfolio\Lunar_Operations_Task_Planning\src\environment\map_generation')
[fig,paths] = create_3D_lunar_base();

% path_keys = keys(paths);
% for i = 1:length(path_keys)
%     path_name = path_keys{i};
%     coords = paths(path_name).coordinates;
%     mid_idx = floor(size(coords,1)/2);
%     label_pos = coords(mid_idx,:);
% 
%     % Add small offset to prevent overlap with path
%     label_pos(3) = label_pos(3) + 2;
% 
%     disp(path_name)
%     % Create text label
%     text(label_pos(1), label_pos(2), label_pos(3), path_name, ...
%         'Color', 'white', ...
%         'FontSize', 12, ...
%         'HorizontalAlignment', 'center', ...
%         'BackgroundColor', 'black', ...
%         'Margin', 1);
% end
% view(0,90)
% 
% exportgraphics(gcf, 'lunar_base_labeled.png', ...
%     'Resolution', 300, ...
%     'BackgroundColor', 'black', ...
%     'ContentType', 'vector');
