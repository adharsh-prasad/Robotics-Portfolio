% close all
% [robot, arm_length] = Robotic_arm_model();
% 
x_func = @(t) (35 + 25*cos(2*pi*t/10))/100;
y_func = @(t) (35 + 25*sin(2*pi*t/10))/100;
z_func = @(t) 90/100*t.^0;
% 
% [T, joint_angles] = Joint_trajectory(x_func, y_func, z_func, robot, arm_length);
% tranform = Forward_Kinematics([arm_length(1) pi/2 0], [0 0 arm_length(2)], [0 0 arm_length(3)]);
% end_effector = [];
% for i = 1:length(joint_angles)
%     joint = joint_angles(i,1:3);
%     theta1 = joint(1);
%     theta2 = joint(2);
%     theta3 = joint(3);
%     end_effector = [end_effector;double(subs(tranform(1:3,4)))'];
% end
figure('Position', get(0, 'Screensize'));  % Create larger figure window
% Plot desired trajectory
plot3(x_func(T), y_func(T), z_func(T), 'b-', 'LineWidth', 2);
hold on

% Plot actual end-effector trajectory
scatter3(end_effector(:,1), end_effector(:,2), end_effector(:,3), 20, 'r.', 'MarkerFaceAlpha', 0.6);

%Enhance plot appearance
grid on
box on
axis equal
xlabel('X (m)', 'FontSize', 12, 'FontWeight', 'bold')
ylabel('Y (m)', 'FontSize', 12, 'FontWeight', 'bold')
zlabel('Z (m)', 'FontSize', 12, 'FontWeight', 'bold')
title('End-Effector Trajectory Tracking', 'FontSize', 14, 'FontWeight', 'bold')

% Add legend
legend('Desired Trajectory', 'Actual Trajectory', 'FontSize', 12, 'Location', 'best')

% Set viewing angle for better visualization
view(45, 30)

% Customize axis limits with some padding
ax = gca;
ax.XLim = [0 1];
ax.YLim = [0 1];
ax.ZLim = [0 1];
ax.LineWidth = 1.5;