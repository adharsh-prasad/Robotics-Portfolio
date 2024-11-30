close all
[robot, arm_length] = Robotic_arm_model();

x_func = @(t) (35 + 25*cos(2*pi*t/10))/100;
y_func = @(t) (35 + 25*sin(2*pi*t/10))/100;
z_func = @(t) 90/100*t.^0;

[T, joint_angles] = Joint_trajectory(x_func, y_func, z_func, robot, arm_length);
tranform = Forward_Kinematics([arm_length(1) pi/2 0], [0 0 arm_length(2)], [0 0 arm_length(3)]);
end_effector = [];
for i = 1:length(joint_angles)
    joint = joint_angles(i,1:3);
    theta1 = joint(1);
    theta2 = joint(2);
    theta3 = joint(3);
    end_effector = [end_effector;double(subs(tranform(1:3,4)))'];
end
% % % figure('Position', get(0, 'Screensize'));  % Create larger figure window
% % % % Plot desired trajectory
% % % plot3(x_func(T), y_func(T), z_func(T), 'b-', 'LineWidth', 2);
% % % hold on
% % % 
% % % % Plot actual end-effector trajectory
% % % scatter3(end_effector(:,1), end_effector(:,2), end_effector(:,3), 20, 'r.', 'MarkerFaceAlpha', 0.6);
% % % 
% % Enhance plot appearance
% % grid on
% % box on
% % axis equal
% % xlabel('X (m)', 'FontSize', 12, 'FontWeight', 'bold')
% % ylabel('Y (m)', 'FontSize', 12, 'FontWeight', 'bold')
% % zlabel('Z (m)', 'FontSize', 12, 'FontWeight', 'bold')
% % title('End-Effector Trajectory Tracking', 'FontSize', 14, 'FontWeight', 'bold')
% % 
% % % Add legend
% % legend('Desired Trajectory', 'Actual Trajectory', 'FontSize', 12, 'Location', 'best')
% % 
% % % Set viewing angle for better visualization
% % view(45, 30)
% % 
% % % Customize axis limits with some padding
% % ax = gca;
% % ax.XLim = [0 1];
% % ax.YLim = [0 1];
% % ax.ZLim = [0 1];
% % ax.LineWidth = 1.5;
% 
fig = figure('WindowState', 'fullscreen', ...
    'MenuBar', 'none', ...
    'ToolBar', 'none', ...
    'NumberTitle', 'off', ...
    'Name', '', ...
    'Units', 'normalized', ...
    'OuterPosition', [0 0 1 1], ...
    'Color', [0.2 0.2 0.2]);

set(fig, 'WindowStyle', 'normal');

% Plot desired trajectory
h1 = plot3(x_func(T), y_func(T), z_func(T), 'b-', 'LineWidth', 2);
hold on

% Configure robot appearance
robot.plotopt = {'noname', 'notiles', 'noshadow', 'nowrist', 'nojaxes', 'nobase'};
robot.plot([0 0 0], 'workspace', [-1 1.5 -1 1.5 -1 1.5]);

% Create empty scatter plot for legend
h2 = scatter3([], [], [], 20, 'g.', 'DisplayName', 'Actual Trajectory');

% Plot settings
box on
grid off
ax = gca;
ax.XGrid = 'off';
ax.YGrid = 'off';
ax.ZGrid = 'off';
axis equal

% Axes properties
ax.Color = [0.5 0.5 0.5];
ax.XColor = [1 1 1];
ax.YColor = [1 1 1];
ax.ZColor = [1 1 1];
ax.GridAlpha = 0.3;
ax.XLim = [-1 1.5];
ax.YLim = [-1 1.5];
ax.ZLim = [-1 1.5];
ax.XTickLabel = [];
ax.YTickLabel = [];
ax.ZTickLabel = [];
ax.XTick = [];
ax.YTick = [];
ax.ZTick = [];

% Labels and title
xlabel('X (m)', 'Color', [1 1 1], 'FontWeight', 'bold')
ylabel('Y (m)', 'Color', [1 1 1], 'FontWeight', 'bold')
zlabel('Z (m)', 'Color', [1 1 1], 'FontWeight', 'bold')
title('Robotic Arm Trajectory Tracking', 'Color', [1 1 1], 'FontWeight', 'bold', 'FontSize', 25)

% Set view
view(45, 30)

% Legend
legend([h1, h2], {'Desired Trajectory', 'Actual Trajectory'}, ...
    'FontSize', 12, ...
    'Location', 'best', ...
    'TextColor', 'white', ...
    'Color', [0.2 0.2 0.2]);
% 
videoPath = 'C:/Users/ADHARSH/Desktop/Job_Search/Projects/Robotic Arm controller/robotic_arm_simulation_with_title.mp4';
v = VideoWriter(videoPath, 'Uncompressed AVI');

% v.FrameRate = 60;  % Increase frame rate for smoother motion
% v.Quality = 100;   % Set maximum quality (0-100)

% Optional: Set figure resolution before recording
% Set figure to 4K resolution
set(gcf, 'Position', [100 100 1920 1080]);  % 4K resolution
% set(gcf, 'GraphicsSmoothing', 'on');
% set(gcf, 'Renderer', 'painters');
open(v);

% Animate robot movement
for i = 1:1:length(joint_angles)
    joint = joint_angles(i,1:3);
    robot.plot(joint, 'trail', 'r-');
    theta1 = joint(1);
    theta2 = joint(2);
    theta3 = joint(3);
    actual_endeffector = double(subs(tranform(1:3,4)));
    scatter3(actual_endeffector(1), actual_endeffector(2), actual_endeffector(3), ...
        20, 'g.', 'HandleVisibility', 'off');
    drawnow
    
    % Capture the frame
    frame = getframe(gcf);
    writeVideo(v, frame);
end
close(v);