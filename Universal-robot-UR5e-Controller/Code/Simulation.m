close all
% Forward Kinematics and Inverse Dynamics
params = Robotic_arm_model();
[transform, theta] = Forward_Kinematics(params);

% Create the robot model
robot = loadrobot('universalUR5e');
t = linspace(0, 10, 100);  % Time vector

% Modified trajectory to be in meters and more realistic workspace
x_func = 0.5 + 0.2*cos(2*pi*t/10);  % Center at 0.5m with 0.2m radius
y_func = 0.2*sin(2*pi*t/10);        % Circle in YZ plane
z_func = 0.5 + 0.2*sin(2*pi*t/10);  % Add height variation

% Initialize desired joint position trajectory
q_d_traj = zeros(length(t), 6);

% Create IK solver for the robot
ik = inverseKinematics('RigidBodyTree', robot);

% For the trajectory calculation loop
initialGuess = homeConfiguration(robot);
for i = 1:length(t)
    position = [x_func(i), y_func(i), z_func(i)];
    
    % Calculate rotation matrix to align end effector with circle normal
    normal = [0 1 -1]/sqrt(2);  % Normal always points along X-axis
    z_axis = normal;
    y_axis = cross([0, 0, 1], z_axis);  % Use global Z as reference
    y_axis = y_axis / norm(y_axis);
    x_axis = cross(y_axis, z_axis);
    
    R = [x_axis', y_axis', z_axis'];  % Rotation matrix
    
    T = [R position'; 0 0 0 1];      
    
    % Weights for position and orientation [x y z R P Y]
    weights = [1 1 1 1 1 1]; % Balance position and orientation
    
    % Solve IK with error checking
    [config, info] = ik('tool0', T, weights, initialGuess);
    if info.ExitFlag < 0
        warning('IK solution not found at time step %d', i);
    end
    q_d_traj(i,:) = [config.JointPosition];
    initialGuess = config;
end

% Compute desired joint velocity trajectory using numerical differentiation
qd_d_traj = zeros(size(q_d_traj));  % Initialize joint velocity trajectory

% Use finite difference method to calculate velocities
dt = t(2) - t(1);  % Time step
for i = 2:length(t)-1
    qd_d_traj(i,:) = (q_d_traj(i+1,:) - q_d_traj(i-1,:)) / (2 * dt);  % Central difference
end

% Handle boundary conditions for the first and last points
qd_d_traj(1,:) = (q_d_traj(2,:) - q_d_traj(1,:)) / dt;      % Forward difference for first point
qd_d_traj(end,:) = (q_d_traj(end,:) - q_d_traj(end-1,:)) / dt;  % Backward difference for last point

% Simulation parameters
Kp = diag([192.1375  927.5531   65.1194  517.4674  955.1177  724.8823]); % Proportional gains
Kd = diag([192.7606  258.5489  906.5033  959.9563  476.4142  751.5266]);    % Derivative gains

% Initial conditions
q0 = q_d_traj(1,:)';
qd0 = zeros(6, 1);

M0 = compute_dynamics_matrices(q0, params);
initial_state = [q0;      % Initial positions
                qd0];     % Initial velocities
% ODE45 call
tspan = linspace(0, 10, 100);
tic
options = odeset('RelTol',1e-3,'AbsTol',1e-6, 'MaxStep', 0.01);
 
[t, state] = ode15s(@(t, state) Robot_dynamics(t, state, q_d_traj, qd_d_traj, Kp, Kd, params, tspan), tspan, initial_state, options);

time = toc;

config = homeConfiguration(robot);

% Modified visualization loop with cinematic styling
fig = figure('WindowState', 'fullscreen', ...
    'MenuBar', 'none', ...
    'ToolBar', 'none', ...
    'NumberTitle', 'off', ...
    'Name', 'UR5e Dynamic Trajectory Tracking', ...
    'Color', [0.85 0.88 0.91], ... % White background
    'Position', [0 0 1920 1080]);

ax = show(robot);
hold on;

% Set axes appearance for a clean, white look
set(ax, 'Color', [0.85 0.88 0.91]);

% axis equal
xlim([-0.5 1.5])
ylim([-1 1])
zlim([0 1.2])
daspect([1 1 1])

% Remove grid and ticks/numbers
grid off
ax.XTick = [];
ax.YTick = [];
ax.ZTick = [];
ax.XTickLabel = [];
ax.YTickLabel = [];
ax.ZTickLabel = [];

% Initialize trajectory containers
actual_path = [];
desired_path = [];

% Add professional annotations
title('UR5e Dynamic Trajectory Tracking', ...
    'Color', 'black', 'FontSize', 24, 'FontWeight', 'bold')
xlabel('X (m)', 'FontSize', 16, 'Color', 'black')
ylabel('Y (m)', 'FontSize', 16, 'Color', 'black')
zlabel('Z (m)', 'FontSize', 16, 'Color', 'black')

% Initialize video writer (optional)
v = VideoWriter('UR5e_simulation.mp4', 'MPEG-4');
v.FrameRate = 30;
v.Quality = 100;

% Define the plane size and position
plane_size = 2; % meters (adjust as needed)
plane_z = 0;      % Z height of the plane (usually 0 for the base)

% Create a grid for the plane
[X, Y] = meshgrid(linspace(-plane_size/2, plane_size/2, 2), ...
                  linspace(-plane_size/2, plane_size/2, 2));
Z = plane_z * ones(size(X));

% Plot the solid plane (floor)
surf(X, Y, Z, ...
    'FaceColor', [0.22 0.22 0.22], ...   % Soft workspace gray-blue
    'EdgeColor', 'none', ...
    'FaceAlpha', 1);                     % Solid (opaque) plane

% Optionally, add a subtle border
plot3(X(1,:), Y(1,:), Z(1,:), 'Color', [0.7 0.7 0.7], 'LineWidth', 1.5);
plot3(X(2,:), Y(2,:), Z(2,:), 'Color', [0.7 0.7 0.7], 'LineWidth', 1.5);
plot3(X(:,1), Y(:,1), Z(:,1), 'Color', [0.7 0.7 0.7], 'LineWidth', 1.5);
plot3(X(:,2), Y(:,2), Z(:,2), 'Color', [0.7 0.7 0.7], 'LineWidth', 1.5);

open(v);

% Set camera to top-angled view
view(-45, 30)  % Azimuth 45 degrees, Elevation 30 degrees

% Add lighting for depth
% light('Position', [1 1 1], 'Style', 'infinite');
% light('Position', [-1 -1 1], 'Style', 'infinite');
lighting gouraud
axis off

for i = 1:length(t)
    % Update robot configuration   
    for j = 1:6
        config(j).JointPosition = state(i,j);
    end
    
    % Update visualization
    show(robot, config, 'Parent', ax, 'PreservePlot', false, ...
        'Visuals', 'on', 'Collisions', 'off', ...
        'FastUpdate', true);
    
    % Store trajectories
    actual_pos = getTransform(robot, config, 'tool0');
    
    % Update trajectory plots
    scatter3(x_func(i), y_func(i), z_func(i), '.', 'r')
    scatter3(actual_pos(1,4), actual_pos(2,4), actual_pos(3,4), '.', 'b')
    
    % Add dynamic timestamp
    timeStamp = text(ax, 0.05, 0.95, 1.1, ...
        sprintf('Time: %.2f s', t(i)), ...
        'Color', 'black', 'FontSize', 14, ...
        'Units', 'normalized');
    
    drawnow limitrate
    frame = getframe(fig);
    writeVideo(v, frame);
    delete(timeStamp); % Remove timestamp for next frame
end

close(v);
