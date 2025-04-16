close all
% Forward Kinematics and Inverse Dynamics
params = Robotic_arm_model();
[transform, theta] = Forward_Kinematics(params);

% MPC Parameters
Ts = 0.02;       % Sampling time
N = 5;       % Prediction steps (N=10)
ns = 12;        % State dimension (6 positions + 6 velocities)
nu = 6;         % Control dimension
xdim = ns*(N);  % Dimension of state part (Q)
udim = nu*N;      % Dimension of control part (U)

% Create the robot model
robot = loadrobot('universalUR5e');
tstart = 0;
tend = 10;
t = linspace(tstart, tend, (tend-tstart)/Ts+1);  % Time vector

% Workspace trajectory in meters
x_func = 0.5 + 0.2*cos(2*pi*t/10);  % Center at 0.5m with 0.2m radius
y_func = 0.2*sin(2*pi*t/10);        % Circle in YZ plane
z_func = 0.5 + 0.2*sin(2*pi*t/10);  % Add height variation

% Trajectory velocity in m/s
v_x_func = -2*pi/10*0.2*sin(2*pi*t/10);  % Center at 0.5m with 0.2m radius
v_y_func = 2*pi/10*0.2*cos(2*pi*t/10);        % Circle in YZ plane
v_z_func = 2*pi/10*0.2*cos(2*pi*t/10);  % Add height variation
velocity = [v_x_func; v_y_func; v_z_func];
velocity = [velocity; zeros(size(velocity))]; % EOF velocity with angular velocity

% Initialize desired joint position and velocity trajectory
q_d_traj = zeros(length(t), 6);
qd_d_traj = zeros(length(t), 6);

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
    current_joint_angle = [config.JointPosition];
    Geometric_Jacobian = compute_geometric_jacobian(current_joint_angle, params);
    q_d_traj(i,:) = current_joint_angle;
    qd_d_traj(i,:) = inv(Geometric_Jacobian)*velocity(:, i);
    initialGuess = config;
end

% Define base weight matrix for position and velocity errors
Q_base = diag([ones(1,6)*0.1, ones(1,6)*100]);  % Higher weight for position errors

% Define decay factor
alpha = 1;  % Decay factor (adjust as needed)

% Construct time-varying weights for each prediction step
Q_blocks = cell(N, 1);
Q_blocks{1} = Q_base;
for i = 1:N-2
    Q_blocks{i+1} = Q_base*alpha;  % Apply decay factor
end

Q_blocks{N} = Q_base*2;

% Combine into a block diagonal matrix
Q_bar = blkdiag(Q_blocks{:});

% Control weights
R = diag([0.0001, 0.0001, 0.0001, 0.0001, 0.001, 0.001]);
R_bar = kron(eye(N), R);

% Combine into full H matrix
H = blkdiag(Q_bar, R_bar);
f = zeros(length(H),1);

% Define bounds with no limits on states, only on controls
lb = [-inf*ones(xdim, 1); repmat(-inf, udim, 1)];
ub = [inf*ones(xdim, 1); repmat(inf, udim, 1)];

% Main simulation loop
% Initial conditions
% q0 = [-0.32   -0.2   -0.9  -0.3   -0.83  -0.4]';
% qd0 = zeros(6,1);

q0 =  q_d_traj(1,:)';
qd0 = qd_d_traj(1,:)';

M0 = compute_dynamics_matrices(q0, params);
initial_state = [q0;      % Initial positions
    qd0];     % Initial velocities
current_state = initial_state;

% Visualization (modified to use state_history)
figure(1);
ax = show(robot);
hold on;
view(3);

config = homeConfiguration(robot);
% Update robot configuration
for j = 1:6
    config(j).JointPosition = initial_state(j);
end

t_current = 0;

current = 1;
Duration = 10;

% Update robot visualization
show(robot, config, 'Parent', ax, 'PreservePlot', false);
actual_pos = getTransform(robot, config, 'tool0');
actual_pos = actual_pos(1:3,4)';

figure(1)
scatter3(x_func(current), y_func(current), z_func(current), '.', 'r');
scatter3(actual_pos(1), actual_pos(2), actual_pos(3), '.', 'b');

q_ref = q_d_traj(current+1:current+N, :);
qd_ref = [qd_d_traj(current+1:current+N, :)];
initq_ref = [zeros(size(q_ref)),qd_ref]';
current_state0 = [initq_ref(:); zeros(udim,1)];

system_current_state = [q_d_traj(1,:)'-current_state(1:6);current_state(7:12)];

while t_current < 10
    % NEW: Get reference trajectory for horizon
    tic
    q_ref = q_d_traj(current:current+N-1, :);
    qd_ref = qd_d_traj(current:current+N-1, :);

    % Linearize dynamics at current state
    [A_d, B_d, G, invM, Cqd] = discretize_dynamics(params, current_state(1:ns/2), current_state(ns/2+1:ns), Ts, ns);

    intial_state_part = A_d*system_current_state;
    W_part = [qd_ref(1,:)';-invM*(G)]*Ts;
    W = intial_state_part + W_part;
    for i = 2:N
        intial_state_part = A_d*intial_state_part;
        W_part = A_d*W_part + [qd_ref(i,:)';-invM*(G)]*Ts;
        W = [W; intial_state_part + W_part];
    end

    W = [W; zeros(nu*N,1)];

    % Build prediction matrices
    [C, F] = build_prediction_matrices(A_d, B_d, N, ns, nu);
    A_bar = [C F; zeros(nu*N, ns*N) zeros(nu*N)];

    % Dynamics based Linear equality constraint
    A_eq = A_bar;
    b_eq = W;
    time = toc;

    % Solve QP
    options = optimoptions('fmincon', 'Display','none'); %'OptimalityTolerance', 1e-6, 'ConstraintTolerance', 1e-3, 'MaxFunctionEvaluations', 100000);
    U_sequence = fmincon(@(U_sequence) my_cost_function(U_sequence, [zeros(size(q_ref)),qd_ref]' , H, udim), current_state0, [], [], A_eq, b_eq, lb, ub, [], options);    

    temp = [zeros(size(q_ref)),qd_ref]';
    current_state_ref = [temp(:); zeros(udim,1)]; 
    % (U_sequence- current_state_ref)' * H * (U_sequence- current_state_ref)

    % Apply first control input
    optimal_torque = U_sequence(xdim+1:xdim+nu);

    % Simulate system
    tspan_local = [t_current t_current+Ts];  
    [~, state_sol] = ode45(@(t,x) Robot_dynamics_MPC(x, optimal_torque, params), tspan_local, [q_d_traj(current, :)';qd_d_traj(current, :)']);

    % Update state
    current_state = state_sol(end, :)';

    % Update robot configuration
    for j = 1:6
        config(j).JointPosition = current_state(j);
    end

    % Update robot visualization
    show(robot, config, 'Parent', ax, 'PreservePlot', false, 'FastUpdate', true);

    actual_pos = getTransform(robot, config, 'tool0');
    actual_pos = actual_pos(1:3,4)';

    % Plot paths
    figure(1)
    scatter3(x_func(current+1), y_func(current+1), z_func(current+1), '.', 'r');
    scatter3(actual_pos(1), actual_pos(2), actual_pos(3), '.', 'b');
    [x_func(current+1), y_func(current+1), z_func(current+1)] - [actual_pos(1), actual_pos(2), actual_pos(3)]

    t_current = t_current + Ts;
    current = current + 1;

    q_ref = q_d_traj(current+1:current+N, :);
    qd_ref = [qd_d_traj(current+1:current+N, :)];
    initq_ref = [zeros(size(q_ref)),qd_ref]';
    current_state0 = [initq_ref(:); U_sequence(xdim+1:end)];  

    system_current_state = [q_d_traj(current,:)'-current_state(1:6);current_state(7:12)];
    drawnow;
end

function cost = my_cost_function(current_state, q_ref, H, udim)
    current_state_ref = [q_ref(:); zeros(udim,1)]; 
    cost = (current_state- current_state_ref)' * H * (current_state- current_state_ref);
end

% MPC Setup
function [A_d, B_d, G, invM, C] = discretize_dynamics(params, q, qd, Ts, ns)
    % Compute continuous-time matrices
    n = ns/2;
    [M, G] = compute_dynamics_matrices(q, params);
    invM = inv(M);
    C = compute_Cqd(q, qd, params, M);
    A_c = [zeros(n,n) -eye(n);
        zeros(n,n) -invM*C];
    B_c = [zeros(n,n); invM];
    
    % Euler discretization
    A_d = eye(ns) + Ts*A_c;
    B_d = Ts*B_c;
end

% NEW: Prediction matrix construction
function [C, F] = build_prediction_matrices(A_d, B_d, N, ns, nu)
    xdim = ns*(N);
    udim = nu*N;
    F = zeros(xdim, udim);
    
    % Precompute A_d powers as a 3D array
    Ad_powers = zeros(ns, ns, N);
    Ad_powers(:,:,1) = A_d; % A_d^1
    
    % Compute higher powers directly
    for k = 2:N
        Ad_powers(:,:,k) = Ad_powers(:,:,k-1) * A_d;
    end
    
    % Store B terms in a 3D array
    B_terms = zeros(ns, nu, N);
    B_terms(:,:,1) = B_d;
    for i = 2:N
        B_terms(:,:,i) = Ad_powers(:,:,i-1) * B_d;
    end
    C = eye(xdim);
    
    for i = 2:N
        row_idx = (i-1)*ns+1:i*ns;
        for j = 1:i-1
            col_idx = (j-1)*nu+1:j*nu;
            F(row_idx, col_idx) = -B_terms(:,:,i-j);
        end
    end
end

function C = compute_Cqd(q, qd, params, M0)
    delta = 1e-6;
    
    % Precompute all mass matrices upfront (7 calls instead of 648)
    M_pert = zeros(6,6,6);  % 3D array: M_pert(:,:,k) = M(q_k + δ)
    
    for k = 1:6
        q_pert = q;
        q_pert(k) = q_pert(k) + delta;
        M_pert(:,:,k) = compute_dynamics_matrices(q_pert, params);
    end
    
    % Compute Christoffel symbols using precomputed matrices
    C = zeros(6,6);
    for i = 1:6
        for j = 1:6
            for k = 1:6
                dMij_dqk = (M_pert(i,j,k) - M0(i,j)) / delta;
                dMik_dqj = (M_pert(i,k,j) - M0(i,k)) / delta;
                dMjk_dqi = (M_pert(j,k,i) - M0(j,k)) / delta;
    
                C(i,j) = C(i,j) + 0.5*(dMij_dqk + dMik_dqj - dMjk_dqi)*qd(k);
            end
        end
    end
end
