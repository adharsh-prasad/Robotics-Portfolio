% ROBOTIC_ARM_SIMULATION - Simulates and visualizes a 3-DOF robotic arm trajectory tracking
%
% This script performs the following:
%   1. Initializes robot model and parameters
%   2. Defines circular trajectory for end-effector
%   3. Solves forward and inverse kinematics
%   4. Creates high-quality visualization
%   5. Generates video recording of the simulation
%
% Required toolboxes:
%   - Robotics Toolbox by Peter Corke
%   - MATLAB R2020a or newer
%
% See also: Robotic_arm_model, Joint_trajectory, Forward_Kinematics

% Clear workspace and figures
close all

% Initialize robot model and get link parameters
[robot, arm_length] = Robotic_arm_model();

% Define circular trajectory functions (normalized to meters)
x_func = @(t) (35 + 25*cos(2*pi*t/10))/100;  % Circular motion in X
y_func = @(t) (35 + 25*sin(2*pi*t/10))/100;  % Circular motion in Y
z_func = @(t) 90/100*t.^0;                   % Constant Z height

% Solve trajectory and get joint angles
[T, joint_angles] = Joint_trajectory(x_func, y_func, z_func, robot, arm_length);

% Calculate forward kinematics transformation
tranform = Forward_Kinematics([arm_length(1) pi/2 0], [0 0 arm_length(2)], [0 0 arm_length(3)]);

% WORKSPACE_ANALYSIS - Generates and visualizes the reachable workspace of the 3-DOF robotic arm

% Initialize workspace points
workspace_points = [];

% Sample joint angles within limits
theta1_range = linspace(-pi, pi, 30);
theta2_range = linspace(-pi/2, pi/2, 30);
theta3_range = linspace(-pi/2, pi/2, 30);

% Generate workspace points through forward kinematics
for theta1 = theta1_range
    for theta2 = theta2_range
        for theta3 = theta3_range
            % Calculate end-effector position for this configuration
            joint = [theta1; theta2; theta3];
            pos = double(subs(tranform(1:3,4)))';
            workspace_points = [workspace_points; pos];
        end
    end
end

view_loop = [40 30;0 0;0 90];
for i = 1:3
    % Create new figure for workspace visualization
    fig = figure('WindowState', 'fullscreen', ...
        'MenuBar', 'none', ...
        'ToolBar', 'none', ...
        'NumberTitle', 'off', ...
        'Name', '', ...
        'Units', 'normalized', ...
        'OuterPosition', [0 0 1 1], ...
        'Color', [0.7 0.7 0.7]);
    scatter3(workspace_points(:,1), workspace_points(:,2), workspace_points(:,3), ...
        20, 'b.', 'MarkerFaceAlpha', 0.3);
    hold on

    % Plot robot base frame for reference
    plot3(0, 0, 0, 'r*', 'MarkerSize', 10)

    % Enhance plot appearance
    title('Reachable Workspace', 'Color', [0 0 0], 'FontWeight', 'bold', 'FontSize', 16)
    xlabel('X (m)', 'Color', [0 0 0], 'FontSize', 13)
    ylabel('Y (m)', 'Color', [0 0 0], 'FontSize', 13)
    zlabel('Z (m)', 'Color', [0 0 0], 'FontSize', 13)

    % Turn grid on
    grid on

    % Set grid spacing
    xticks(-1:0.2:1);
    yticks(-1:0.2:1);
    zticks(-0.5:0.2:1.5);

    daspect([1 1 1]);
    xlim([-1 1]);
    ylim([-1 1]);
    zlim([-0.5 1.5]);
    view(view_loop(i,1), view_loop(i,2))
end

% Initialize end-effector position array
end_effector = [];

% Calculate end-effector positions for each time step
for i = 1:length(joint_angles)
    joint = joint_angles(i,1:3);
    theta1 = joint(1);
    theta2 = joint(2);
    theta3 = joint(3);
    end_effector = [end_effector;double(subs(tranform(1:3,4)))'];
end

% Create figure with specific styling
fig = figure('WindowState', 'fullscreen', ...
    'MenuBar', 'none', ...
    'ToolBar', 'none', ...
    'NumberTitle', 'off', ...
    'Name', '', ...
    'Units', 'normalized', ...
    'OuterPosition', [0 0 1 1], ...
    'Color', [0.2 0.2 0.2]);  % Dark theme background

set(fig, 'WindowStyle', 'normal');

% Plot desired trajectory path
h1 = plot3(x_func(T), y_func(T), z_func(T), 'b-', 'LineWidth', 2);
hold on

% Configure robot visualization settings
robot.plotopt = {'noname', 'notiles', 'noshadow', 'nowrist', 'nojaxes', 'nobase'};
robot.plot([0 0 0], 'workspace', [-1 1.5 -1 1.5 -1 1.5]);

% Initialize actual trajectory plot
h2 = scatter3([], [], [], 20, 'g.', 'DisplayName', 'Actual Trajectory');

% Configure plot aesthetics
box on
grid off
ax = gca;
ax.XGrid = 'off';
ax.YGrid = 'off';
ax.ZGrid = 'off';
axis equal

% Set axes properties for professional appearance
ax.Color = [0.5 0.5 0.5];        % Gray background
ax.XColor = [1 1 1];             % White axes
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

% Add labels and title
xlabel('X (m)', 'Color', [1 1 1], 'FontWeight', 'bold')
ylabel('Y (m)', 'Color', [1 1 1], 'FontWeight', 'bold')
zlabel('Z (m)', 'Color', [1 1 1], 'FontWeight', 'bold')
title('Robotic Arm Trajectory Tracking', 'Color', [1 1 1], 'FontWeight', 'bold', 'FontSize', 25)

% Set optimal viewing angle
view(45, 30)

% Add legend with custom styling
legend([h1, h2], {'Desired Trajectory', 'Actual Trajectory'}, ...
    'FontSize', 12, ...
    'Location', 'best', ...
    'TextColor', 'white', ...
    'Color', [0.2 0.2 0.2]);

% Initialize video writer
videoPath = 'robotic_arm_simulation_with_title.mp4';
v = VideoWriter(videoPath, 'Uncompressed AVI');

% Set figure resolution for high-quality recording
set(gcf, 'Position', [100 100 1920 1080]);  % 1080p resolution
open(v);

% Animation loop
for i = 1:1:length(joint_angles)
    % Update robot configuration
    joint = joint_angles(i,1:3);
    robot.plot(joint, 'trail', 'r-');

    % Extract joint angles
    theta1 = joint(1);
    theta2 = joint(2);
    theta3 = joint(3);

    % Plot actual end-effector position
    actual_endeffector = double(subs(tranform(1:3,4)));
    scatter3(actual_endeffector(1), actual_endeffector(2), actual_endeffector(3), ...
        20, 'g.', 'HandleVisibility', 'off');
    drawnow

    % Capture and write video frame
    frame = getframe(gcf);
    writeVideo(v, frame);
end

% Close video writer
close(v);