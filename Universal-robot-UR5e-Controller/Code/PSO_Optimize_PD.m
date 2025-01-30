function [GBest, GBest_history] = PSO_Optimize_PD()
    % Robot initialization
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
    
    % PSO Parameters
    n_particles = 50;       % Reduced for faster computation
    max_iter = 300;          % Reduced iterations for initial testing
    dim = 12;               % 6 Kp + 6 Kd values
    bounds = [0 1000;       % Kp bounds
        0 1000];       % Kd bounds
    
    % Initialize PSO
    particles = initialize_particles(n_particles, dim, bounds);
    velocity = zeros(n_particles, dim);
    
    % PSO coefficients
    inertia = 0.8;
    c1 = 1.5;
    c2 = 1.5;
    
    % Optimization loop
    GBest = inf(1,dim);
    GBest_fitness = inf;
    PBest = particles;
    PBest_fitness = inf(n_particles,1);
    
    options = odeset('RelTol',1e-2,'AbsTol',1e-3, 'MaxStep', 0.01);
    tspan = linspace(0, 10, 20);
    initial_state = [[0, -pi/2, 0, -pi/2, 0, 0], zeros(1,6)]; 
    
    disp("loop started")
    for iter = 1:max_iter
        disp(particles(i, :))
        % Evaluate all particles
        fitness = zeros(n_particles,1);
        for i = 1:n_particles
            Kp = diag(particles(i,1:6));
            Kd = diag(particles(i,7:12));
   
            disp(particles(i, :))
            % Simulate with current gains
            tic
            [t, state] = ode15s(@(t,state) Robot_dynamics(t, state, q_d_traj, qd_d_traj, Kp, Kd, params, tspan), tspan, initial_state, options);
            time = toc;
            disp(time)
    
            % Calculate fitness (tracking error)
            fitness(i) = calculate_tracking_error(t, state, q_d_traj);
            disp(fitness(i))
        end
        disp("particle done")
    
        % Update personal best
        improved = fitness < PBest_fitness;
        PBest(improved,:) = particles(improved,:);
        PBest_fitness(improved) = fitness(improved);
    
        % Update global best
        [min_fitness, idx] = min(fitness);
        if min_fitness < GBest_fitness
            GBest = particles(idx,:);
            GBest_fitness = min_fitness;
        end
    
        % Update particle velocities and positions
        velocity = inertia*velocity + ...
            c1*rand(n_particles,dim).*(PBest - particles) + ...
            c2*rand(n_particles,dim).*(GBest - particles);
        particles = particles + velocity;
    
        % Apply bounds
        particles(:,1:6) = min(max(particles(:,1:6), bounds(1,1)), bounds(1,2));
        particles(:,7:12) = min(max(particles(:,7:12), bounds(2,1)), bounds(2,2));
    
        % Store best fitness
        GBest_history(iter) = GBest_fitness;
        disp(iter)
    end
    
    % Final validation with best parameters
    Kp_opt = diag(GBest(1:6));
    Kd_opt = diag(GBest(7:12));
    [t, state] = simulate_robot(Kp_opt, Kd_opt, q_d_traj, qd_d_traj, params);
    plot_results(t, state, q_d_traj);
end

function [q_d_traj, qd_d_traj] = generate_trajectory(num_points, robot, params)
    t = linspace(0, 10, num_points);
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
end

function [t, state] = simulate_robot(Kp, Kd, q_d_traj, qd_d_traj, params)
options = odeset('RelTol',1e-2,'AbsTol',1e-3, 'MaxStep',0.01);
tspan = linspace(0, 10, 20);
initial_state = [zeros(6,1); zeros(6,1)];

[t, state] = ode15s(@(t,state) Robot_dynamics(t, state, q_d_traj, qd_d_traj, Kp, Kd, params, tspan),...
    tspan, initial_state, options);
end

function error = calculate_tracking_error(t, state, q_d_traj)
actual_q = state(:,1:6);
error = mean(sqrt(sum((actual_q - q_d_traj).^2, 2)));
end

function particles = initialize_particles(n_particles, dim, bounds)
particles = zeros(n_particles, dim);
particles(:,1:6) = bounds(1,1) + (bounds(1,2)-bounds(1,1))*rand(n_particles,6);
particles(:,7:12) = bounds(2,1) + (bounds(2,2)-bounds(2,1))*rand(n_particles,6);
end
