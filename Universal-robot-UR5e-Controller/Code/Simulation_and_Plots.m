% % Set up the robotic arm simulation (as in the provided code)
% close all
% [robot, arm_length] = Robotic_arm_model();  % Robot model
ur5e = loadrobot('universalUR5e', 'DataFormat', 'row', 'Gravity', [0, 0, -9.81]);

% Desired Trajectory
x_func = @(t) (70 + 25*cos(2*pi*t/10))/100;
y_func = @(t) (70 + 25*sin(2*pi*t/10))/100;
z_func = @(t) 90/100*t.^0;

syms t
q_d_sym = Inverse_Dynamics([x_func(t); y_func(t); z_func(t)], robot);
qd_d_sym = diff(q_d_sym);

% Convert symbolic expressions to MATLAB functions
q_d_func = matlabFunction(q_d_sym, 'Vars', t);
qd_d_func = matlabFunction(qd_d_sym, 'Vars', t);

% 
% % Use the optimized PD gains (GBest from PSO)
% % [GBest, GBest_history, PBest_avg_history] = PSO_Optimize_PD();
% Kp = GBest(1:3);  % Optimized proportional gains
% Kd = GBest(4:6);  % Optimized derivative gains
% 
% % Desired trajectory functions (as defined earlier)
% x_func = @(t) (35 + 25*cos(2*pi*t/10))/100;
% y_func = @(t) (35 + 25*sin(2*pi*t/10))/100;
% z_func = @(t) 90/100*t.^0;
% 
% % Forward Kinematics and Inverse Dynamics (as defined earlier)
% transform = Forward_Kinematics([arm_length(1) pi/2 0], [0 0 arm_length(2)], [0 0 arm_length(3)]);
% 
% % Robots Dynamics Matrices
% syms q1 q2 q3 qd1 qd2 qd3 real
% q_sym = [q1; q2; q3];
% qd_sym = [qd1; qd2; qd3];
% 
% M_sym = robot.inertia(q_sym');
% C_sym = robot.coriolis(q_sym', qd_sym');
% G_sym = robot.gravload(q_sym');
% 
% % Convert symbolic expressions to MATLAB functions
% M_func = matlabFunction(M_sym, 'Vars', {q1, q2, q3});
% C_func = matlabFunction(C_sym, 'Vars', {q1, q2, q3, qd1, qd2, qd3});
% G_func = matlabFunction(G_sym, 'Vars', {q1, q2, q3});
% 
% % Simulation parameters
% t_span = [0 10];  % Time span for simulation
% initial_state = [30*pi/180; -30*pi/180; 80*pi/180; 0; 0; 0]; % Initial joint angles and velocities
% [t, state] = ode45(@(t, state) Robot_dynamics(t, state, q_d_func, qd_d_func, Kp, Kd, M_func, C_func, G_func), t_span, initial_state);
% 
% % Create figure with specific styling
% fig = figure('WindowState', 'fullscreen', ...
%     'MenuBar', 'none', ...
%     'ToolBar', 'none', ...
%     'NumberTitle', 'off', ...
%     'Name', '', ...
%     'Units', 'normalized', ...
%     'OuterPosition', [0 0 1 1], ...
%     'Color', [0.2 0.2 0.2]);  % Dark theme background
% 
% set(fig, 'WindowStyle', 'normal');
% 
% h2 = scatter3([], [], [], 20, 'g.', 'DisplayName', 'Actual Trajectory');
% hold on
% 
% % Initialize visualization
% robot.plotopt = {'noname', 'notiles', 'noshadow', 'nowrist', 'nojaxes', 'nobase'};
% robot.plot([0 0 0], 'workspace', [-1 1.5 -1 1.5 -1 1.5]);
% 
% % Initialize actual trajectory plot
% T = linspace(0,10,200);
% h1 = plot3(x_func(T), y_func(T), z_func(T), 'r-', 'LineWidth', 2, 'DisplayName', 'Desired Trajectory'); 
% 
% % Plot aesthetics
% box on
% grid off
% ax = gca;
% ax.XGrid = 'off';
% ax.YGrid = 'off';
% ax.ZGrid = 'off';
% axis equal
% ax.Color = [0.5 0.5 0.5];
% ax.XColor = [1 1 1];
% ax.YColor = [1 1 1];
% ax.ZColor = [1 1 1];
% ax.GridAlpha = 0.3;
% ax.XLim = [-1 1.5];
% ax.YLim = [-1 1.5];
% ax.ZLim = [-1 1.5];
% ax.XTickLabel = [];
% ax.YTickLabel = [];
% ax.ZTickLabel = [];
% ax.XTick = [];
% ax.YTick = [];
% ax.ZTick = [];
% 
% % Add labels and title
% xlabel('X (m)', 'Color', [1 1 1], 'FontWeight', 'bold')
% ylabel('Y (m)', 'Color', [1 1 1], 'FontWeight', 'bold')
% zlabel('Z (m)', 'Color', [1 1 1], 'FontWeight', 'bold')
% title('Robotic Arm Trajectory Tracking', 'Color', [1 1 1], 'FontWeight', 'bold', 'FontSize', 25)
% 
% % Set optimal viewing angle
% view(45, 30)
% 
% % Add legend with custom styling
% legend([h1, h2], {'Desired Trajectory', 'Actual Trajectory'}, ...
%     'FontSize', 12, ...
%     'Location', 'best', ...
%     'TextColor', 'white', ...
%     'Color', [0.2 0.2 0.2]);
% 
% % Initialize video writer
% videoPath = 'robotic_arm_simulation_with_optimized_controller.mp4';
% v = VideoWriter(videoPath, 'Uncompressed AVI');
% set(gcf, 'Position', [100 100 1920 1080]);  % Set resolution for high-quality video
% open(v);
% 
% % Animation loop
% for i = 1:4:length(state)
%     % Update robot configuration
%     joint = state(i,1:3);
%     robot.plot(joint, 'trail', 'r-');
% 
%     % Extract joint angles
%     theta1 = joint(1);
%     theta2 = joint(2);
%     theta3 = joint(3);
% 
%     % Plot actual end-effector position
%     actual_endeffector = double(subs(transform(1:3,4)));
%     scatter3(actual_endeffector(1), actual_endeffector(2), actual_endeffector(3), 20, 'g.', 'HandleVisibility', 'off');
%     drawnow
% 
%     % Capture and write video frame
%     frame = getframe(gcf);
%     writeVideo(v, frame);
% end
% 
% % Close video writer
% close(v);
% 
% %Convergence plot
% num_iterations = 300;
% figure(2);
% plot(0:num_iterations, GBest_history, 'b-', 'LineWidth', 2);
% hold on;
% plot(0:num_iterations, PBest_avg_history, 'r--', 'LineWidth', 1.5);
% xlabel('Iteration');
% ylabel('RMS Error');
% title('PSO Convergence for PD Controller Optimization');
% legend('Global Best Fitness', 'Particles Mean Fitness');
% 
% % Helper function for robot dynamics (you need to ensure this is properly modified for PD control)
% function dstate = Robot_dynamics(t, state, q_d_func, qd_d_func, Kp, Kd, M_Matrix, C_Matrix, G_Matrix)
%     q = state(1:3);  % Joint angles
%     qd = state(4:6);  % Joint velocities
% 
%     % Desired trajectory and velocity
%     q_d = q_d_func(t);
%     qd_d = qd_d_func(t);
% 
%     % Compute error terms
%     e = q_d - q;  % Position error
%     ed = qd_d - qd;  % Velocity error
% 
%     % PD control
%     tau = Kp' .* e + Kd' .* ed;  % Control torques
% 
%     % Compute the robot's dynamics (inertia, Coriolis, and gravity)
%     M = M_Matrix(q(1), q(2), q(3));
%     C = C_Matrix(q(1), q(2), q(3), qd(1), qd(2), qd(3));
%     G = G_Matrix(q(1), q(2), q(3));
% 
%     % Compute joint accelerations
%     qdd = M\(tau - C*qd - G');  % Solve for joint accelerations
%     dstate = [qd; qdd];  % Return joint velocities and accelerations
% end