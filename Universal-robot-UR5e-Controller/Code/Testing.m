% % Initial conditions
q0 = zeros(6, 1);
qd0 = zeros(6, 1);
initial_state = [q0; qd0];

% Forward Kinematics and Inverse Dynamics (as defined earlier)
params = Robotic_arm_model();
transform = Forward_Kinematics(params);

% Create the robot model
robot = loadrobot('universalUR5e');
% Define desired end-effector trajectory (e.g., circular path)
t = linspace(0, 10, 20);  % Time vector

x_func = (80*cos(2*pi*t/10))/100;
y_func = (80*sin(2*pi*t/10))/100;
z_func = 15/100*t.^0;


% Initialize desired joint position trajectory
q_d_traj = zeros(length(t), 6);

% Compute desired joint angles using inverse kinematics
for i = 1:length(t)
    position = [x_func(i), y_func(i), z_func(i)];
    jointAngles = Inverse_Dynamics(position, robot, params);
    q_d_traj(i,:) = jointAngles.JointPosition;  % Solve inverse kinematics
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

% Now q_d_traj and qd_d_traj are ready to be used in your Robot_dynamics_optimized function.

% Simulation parameters
tspan = [0 10];  % Time span for simulation
Kp = diag([192.1375  927.5531   65.1194  517.4674  955.1177  724.8823]); % Proportional gains
Kd = diag([19.7606  258.5489  906.5033  959.9563  476.4142  751.5266]);    % Derivative gains

% ODE45 call

% Initial conditions
q0 = zeros(6, 1); n=6;
qd0 = zeros(6, 1);

M0 = compute_dynamics_matrices(q0, params);
initial_state = [q0;           % Initial joint positions
                qd0];          % Initial joint velocities

t0 = 0;
initial_state = [q0; qd0; t0];
% Time span
tspan = [0 10];

initial_state = [q0;      % Initial positions
                qd0];     % Initial velocities
% ODE45 call
% tes
tspan = linspace(0, 10, 20);
tic
options = odeset('RelTol',1e-2,'AbsTol',1e-3, 'MaxStep', 0.01);
[t, state] = ode15s(@(t, state) Robot_dynamics(t, state, q_d_traj, qd_d_traj, Kp, Kd, params, tspan), tspan, initial_state, options);
time = toc;