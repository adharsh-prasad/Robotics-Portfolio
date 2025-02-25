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
% Modified visualization loop
figure;
ax = show(robot);
hold on;
view(3);  % Set 3D view

config = homeConfiguration(robot);
actual_path = [];
desired_path = [];

for i = 1:length(t)
    % Update robot configuration   
    for j = 1:6
        config(j).JointPosition = state(i,j);
    end
    
    % Update robot visualization
    show(robot, config, 'Parent', ax, 'PreservePlot', false);
    
    % Store and plot trajectories
    desired_path = [desired_path; x_func(i) y_func(i) z_func(i)];
    actual_pos = getTransform(robot, config, 'tool0');
    actual_path = [actual_path; actual_pos(1:3,4)'];
    
    % Plot paths
    plot3(desired_path(:,1), desired_path(:,2), desired_path(:,3), 'r-', 'LineWidth', 2);
    plot3(actual_path(:,1), actual_path(:,2), actual_path(:,3), 'b-', 'LineWidth', 2);
    
    drawnow;
    pause(0.01);  % Smooth visualization
end
