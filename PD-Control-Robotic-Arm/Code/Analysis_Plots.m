close all
% [robot, arm_length] = Robotic_arm_model();
%
x_func = @(t) (35 + 25*cos(2*pi*t/10))/100;
y_func = @(t) (35 + 25*sin(2*pi*t/10))/100;
z_func = @(t) 90/100*t.^0;

% Define symbolic variable for time
syms t

% Generate desired joint trajectories using inverse kinematics
q_d_sym = Inverse_Dynamics([x_func(t); y_func(t); z_func(t)], arm_length(1), arm_length(2), arm_length(3))';

% Compute velocity and acceleration profiles
qd_d_sym = diff(q_d_sym);    % First derivative (velocity)
qdd_d_sym = diff(qd_d_sym);  % Second derivative (acceleration)

% Convert symbolic expressions to MATLAB functions
q_d_func = matlabFunction(q_d_sym, 'Vars', t);
qd_d_func = matlabFunction(qd_d_sym, 'Vars', t);
qdd_d_func = matlabFunction(qdd_d_sym, 'Vars', t);


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

view_loop = [40 30;0 0;0 90];
for i = 1:3

    fig = figure('WindowState', 'fullscreen', ...
        'MenuBar', 'none', ...
        'ToolBar', 'none', ...
        'NumberTitle', 'off', ...
        'Name', '', ...
        'Units', 'normalized', ...
        'OuterPosition', [0 0 1 1], ...
        'Color', [0.7 0.7 0.7]);  % Light gray background

    % Plot desired trajectory
    h1 = plot3(x_func(T), y_func(T), z_func(T), 'b-', 'LineWidth', 2);
    hold on

    % Plot actual end-effector trajectory
    h2 = scatter3(end_effector(:,1), end_effector(:,2), end_effector(:,3), ...
        20, 'r.', 'MarkerFaceAlpha', 0.6);

    % Configure plot aesthetics
    box on
    grid on
    axis equal

    % Set axes properties for professional appearance
    ax = gca;
    ax.Color = [0.7 0.7 0.7];        % Light gray background
    ax.XColor = [0 0 0];             % Black axes
    ax.YColor = [0 0 0];
    ax.ZColor = [0 0 0];
    ax.GridAlpha = 0.3;
    ax.LineWidth = 1.5;

    % Set grid spacing
    xticks(-1:0.2:1);
    yticks(-1:0.2:1);
    zticks(-0.5:0.2:1.5);

    % Set axis limits
    xlim([-1 1]);
    ylim([-1 1]);
    zlim([-0.5 1.5]);

    % Maintain aspect ratio
    daspect([1 1 1]);

    % Add labels and title
    xlabel('X (m)', 'Color', [0 0 0], 'FontWeight', 'bold', 'FontSize', 13)
    ylabel('Y (m)', 'Color', [0 0 0], 'FontWeight', 'bold', 'FontSize', 13)
    zlabel('Z (m)', 'Color', [0 0 0], 'FontWeight', 'bold', 'FontSize', 13)
    title('End-Effector Trajectory Tracking', 'Color', [0 0 0], ...
        'FontWeight', 'bold', 'FontSize', 16)

    % Set viewing angle
    view(view_loop(i,1), view_loop(i,2))

    % Add legend
    legend([h1, h2], {'Desired Trajectory', 'Actual Trajectory'}, ...
        'FontSize', 12, ...
        'Location', 'best', ...
        'Color', [0.7 0.7 0.7]);
    exportgraphics(gcf, 'eof_trajectory_tracking'+string(i)+'.jpg', 'Resolution', 300, 'BackgroundColor', 'current')
end

% First Figure: End-Effector Position Tracking
fig = figure('WindowState', 'fullscreen', ...
    'MenuBar', 'none', ...
    'ToolBar', 'none', ...
    'NumberTitle', 'off', ...
    'Name', '', ...
    'Units', 'normalized', ...
    'OuterPosition', [0 0 1 1], ...
    'Color', [0.7 0.7 0.7]);

% Plot X position
subplot(3,1,1)
plot(T, end_effector(:,1), 'b-', 'LineWidth', 2)
hold on
plot(T, x_func(T), 'r--', 'LineWidth', 2)
grid on
ylabel('X Position (m)', 'FontWeight', 'bold', 'FontSize', 13)
title('End-Effector Position vs Time', 'FontWeight', 'bold', 'FontSize', 16)
legend('Actual', 'Desired', 'Location', 'best')

% Plot Y position
subplot(3,1,2)
plot(T, end_effector(:,2), 'b-', 'LineWidth', 2)
hold on
plot(T, y_func(T), 'r--', 'LineWidth', 2)
grid on
ylabel('Y Position (m)', 'FontWeight', 'bold', 'FontSize', 13)

% Plot Z position
subplot(3,1,3)
plot(T, end_effector(:,3), 'b-', 'LineWidth', 2)
hold on
plot(T, z_func(T), 'r--', 'LineWidth', 2)
grid on
xlabel('Time (s)', 'FontWeight', 'bold', 'FontSize', 13)
ylabel('Z Position (m)', 'FontWeight', 'bold', 'FontSize', 13)

% Enhance plot appearance
set(gcf, 'Position', [100 100 1920 1080])
set(findall(gcf,'-property','FontSize'),'FontSize', 12)

% Save the figure
exportgraphics(gcf, 'end_effector_position.png', 'Resolution', 300)

% Second Figure: Joint Angles Plot
fig = figure('WindowState', 'fullscreen', ...
    'MenuBar', 'none', ...
    'ToolBar', 'none', ...
    'NumberTitle', 'off', ...
    'Name', '', ...
    'Units', 'normalized', ...
    'OuterPosition', [0 0 1 1], ...
    'Color', [0.7 0.7 0.7]);

% Get desired joint angles
desired_joint_angles = q_d_func(T);

% Plot first joint angle
subplot(3,1,1)
plot(T, joint_angles(:,1)*180/pi, 'b-', 'LineWidth', 2)
hold on
plot(T, desired_joint_angles(:,1)*180/pi, 'r--', 'LineWidth', 2)
grid on
ylabel('\theta_1 (deg)', 'FontWeight', 'bold', 'FontSize', 13)
title('Joint Angles vs Time', 'FontWeight', 'bold', 'FontSize', 16)
legend('Actual', 'Desired', 'Location', 'best')

% Plot second joint angle
subplot(3,1,2)
plot(T, joint_angles(:,2)*180/pi, 'b-', 'LineWidth', 2)
hold on
plot(T, desired_joint_angles(:,2)*180/pi, 'r--', 'LineWidth', 2)
grid on
ylabel('\theta_2 (deg)', 'FontWeight', 'bold', 'FontSize', 13)

% Plot third joint angle
subplot(3,1,3)
plot(T, joint_angles(:,3)*180/pi, 'b-', 'LineWidth', 2)
hold on
plot(T, desired_joint_angles(:,3)*180/pi, 'r--', 'LineWidth', 2)
grid on
xlabel('Time (s)', 'FontWeight', 'bold', 'FontSize', 13)
ylabel('\theta_3 (deg)', 'FontWeight', 'bold', 'FontSize', 13)

% Enhance plot appearance
set(gcf, 'Position', [100 100 1920 1080])  % Set to 1080p resolution
set(findall(gcf,'-property','FontSize'),'FontSize', 12)

% Save the figure
exportgraphics(gcf, 'joint_angles.png', 'Resolution', 300)

% Third Figure: Joint Angular Velocities Plot
fig = figure('WindowState', 'fullscreen', ...
    'MenuBar', 'none', ...
    'ToolBar', 'none', ...
    'NumberTitle', 'off', ...
    'Name', '', ...
    'Units', 'normalized', ...
    'OuterPosition', [0 0 1 1], ...
    'Color', [0.7 0.7 0.7]);

% Get desired joint angular velocities
desired_joint_angular_velocity = qd_d_func(T);

% Plot first joint angular velocity
subplot(3,1,1)
plot(T, joint_angles(:,4)*180/pi, 'b-', 'LineWidth', 2)
hold on
plot(T, desired_joint_angular_velocity(:,1)*180/pi, 'r--', 'LineWidth', 2)
grid on
ylabel('\omega_1 (deg/s)', 'FontWeight', 'bold', 'FontSize', 13)
title('Joint Angular Velocities vs Time', 'FontWeight', 'bold', 'FontSize', 16)
legend('Actual', 'Desired', 'Location', 'best')

% Plot second joint angular velocity
subplot(3,1,2)
plot(T, joint_angles(:,5)*180/pi, 'b-', 'LineWidth', 2)
hold on
plot(T, desired_joint_angular_velocity(:,2)*180/pi, 'r--', 'LineWidth', 2)
grid on
ylabel('\omega_2 (deg/s)', 'FontWeight', 'bold', 'FontSize', 13)

% Plot third joint angular velocity
subplot(3,1,3)
plot(T, joint_angles(:,6)*180/pi, 'b-', 'LineWidth', 2)
hold on
plot(T, desired_joint_angular_velocity(:,3)*180/pi, 'r--', 'LineWidth', 2)
grid on
xlabel('Time (s)', 'FontWeight', 'bold', 'FontSize', 13)
ylabel('\omega_3 (deg/s)', 'FontWeight', 'bold', 'FontSize', 13)

% Enhance plot appearance
set(gcf, 'Position', [100 100 1920 1080])  % Set to 1080p resolution
set(findall(gcf,'-property','FontSize'),'FontSize', 12)

% Save the figure
exportgraphics(gcf, 'joint_angular_velocities.png', 'Resolution', 300)

% After ODE45 solution, calculate torques for each time step
tau_history = zeros(3, length(T));

for i = 1:length(T)
    % Get current state
    q = joint_angles(i,1:3);
    qd = joint_angles(i,4:6);

    % Get desired trajectory at current time
    q_d = q_d_func(T(i));
    qd_d = qd_d_func(T(i));

    % Calculate control input (PD control)
    Kp = diag([80, 80, 80]);
    Kd = diag([15, 15, 15]);
    e = q_d - q;
    ed = qd_d - qd;
    tau = Kp*e' + Kd*ed';

    % Store torque values
    tau_history(:,i) = tau;
end

% Create figure with specific styling
fig = figure('WindowState', 'fullscreen', ...
    'MenuBar', 'none', ...
    'ToolBar', 'none', ...
    'NumberTitle', 'off', ...
    'Name', '', ...
    'Units', 'normalized', ...
    'OuterPosition', [0 0 1 1], ...
    'Color', [0.7 0.7 0.7]);

% Plot torques for each joint
subplot(3,1,1)
plot(T, tau_history(1,:), 'b-', 'LineWidth', 2)
grid on
ylabel('τ_1 (Nm)', 'FontWeight', 'bold', 'FontSize', 13)
title('Joint Torques vs Time', 'FontWeight', 'bold', 'FontSize', 16)

subplot(3,1,2)
plot(T, tau_history(2,:), 'g-', 'LineWidth', 2)
grid on
ylabel('τ_2 (Nm)', 'FontWeight', 'bold', 'FontSize', 13)

subplot(3,1,3)
plot(T, tau_history(3,:), 'r-', 'LineWidth', 2)
grid on
xlabel('Time (s)', 'FontWeight', 'bold', 'FontSize', 13)
ylabel('τ_3 (Nm)', 'FontWeight', 'bold', 'FontSize', 13)

% Enhance plot appearance
set(gcf, 'Position', [100 100 1920 1080])
set(findall(gcf,'-property','FontSize'),'FontSize', 12)

% Save the figure
exportgraphics(gcf, 'control_torques.png', 'Resolution', 300)

% Create figure with specific styling
fig = figure('WindowState', 'fullscreen', ...
    'MenuBar', 'none', ...
    'ToolBar', 'none', ...
    'NumberTitle', 'off', ...
    'Name', '', ...
    'Units', 'normalized', ...
    'OuterPosition', [0 0 1 1], ...
    'Color', [0.7 0.7 0.7]);

% Calculate errors
joint_errors = zeros(length(T), 3);
for i = 1:length(T)
    q_d = q_d_func(T(i));
    joint_errors(i,:) = (joint_angles(i,1:3) - q_d)'*180/pi;  % Convert to degrees
end

% Plot joint angle errors
subplot(3,1,1)
plot(T, joint_errors(:,1), 'b-', 'LineWidth', 2)
grid on
ylabel('e_1 (deg)', 'FontWeight', 'bold', 'FontSize', 13)
title('Joint Angle Tracking Errors', 'FontWeight', 'bold', 'FontSize', 16)

subplot(3,1,2)
plot(T, joint_errors(:,2), 'g-', 'LineWidth', 2)
grid on
ylabel('e_2 (deg)', 'FontWeight', 'bold', 'FontSize', 13)

subplot(3,1,3)
plot(T, joint_errors(:,3), 'r-', 'LineWidth', 2)
grid on
xlabel('Time (s)', 'FontWeight', 'bold', 'FontSize', 13)
ylabel('e_3 (deg)', 'FontWeight', 'bold', 'FontSize', 13)

% Enhance plot appearance
set(gcf, 'Position', [100 100 1920 1080])
set(findall(gcf,'-property','FontSize'),'FontSize', 12)

% Save the figure
exportgraphics(gcf, 'joint_angle_tracking_error.png', 'Resolution', 300)

% Create figure with specific styling
fig = figure('WindowState', 'fullscreen', ...
    'MenuBar', 'none', ...
    'ToolBar', 'none', ...
    'NumberTitle', 'off', ...
    'Name', '', ...
    'Units', 'normalized', ...
    'OuterPosition', [0 0 1 1], ...
    'Color', [0.7 0.7 0.7]);

% Calculate velocity errors
velocity_errors = zeros(length(T), 3);
for i = 1:length(T)
    qd_d = qd_d_func(T(i));
    velocity_errors(i,:) = (joint_angles(i,4:6) - qd_d)'*180/pi;  % Convert to deg/s
end

% Plot velocity errors
subplot(3,1,1)
plot(T, velocity_errors(:,1), 'b-', 'LineWidth', 2)
grid on
ylabel('ω_1 error (deg/s)', 'FontWeight', 'bold', 'FontSize', 13)
title('Joint Angular Velocity Errors', 'FontWeight', 'bold', 'FontSize', 16)

subplot(3,1,2)
plot(T, velocity_errors(:,2), 'g-', 'LineWidth', 2)
grid on
ylabel('ω_2 error (deg/s)', 'FontWeight', 'bold', 'FontSize', 13)

subplot(3,1,3)
plot(T, velocity_errors(:,3), 'r-', 'LineWidth', 2)
grid on
xlabel('Time (s)', 'FontWeight', 'bold', 'FontSize', 13)
ylabel('ω_3 error (deg/s)', 'FontWeight', 'bold', 'FontSize', 13)

% Enhance plot appearance
set(gcf, 'Position', [100 100 1920 1080])
set(findall(gcf,'-property','FontSize'),'FontSize', 12)

% Save the figure
exportgraphics(gcf, 'joint_angular_velocity_errors.png', 'Resolution', 300)
