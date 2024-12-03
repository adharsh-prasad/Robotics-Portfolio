% Initialize the robotic arm
[robot, arm_length] = Robotic_arm_model();

% Desired Trajectory
x_func = @(t) (35 + 25*cos(2*pi*t/10))/100;
y_func = @(t) (35 + 25*sin(2*pi*t/10))/100;
z_func = @(t) 90/100*t.^0;

syms t
q_d_sym = Inverse_Dynamics([x_func(t); y_func(t); z_func(t)], arm_length(1), arm_length(2), arm_length(3));
qd_d_sym = diff(q_d_sym);

% Convert symbolic expressions to MATLAB functions
q_d_func = matlabFunction(q_d_sym, 'Vars', t);
qd_d_func = matlabFunction(qd_d_sym, 'Vars', t);

% Transformation Matrix
transform = Forward_Kinematics([arm_length(1) pi/2 0], [0 0 arm_length(2)], [0 0 arm_length(3)]);
transform = matlabFunction(transform(1:3,4)', 'Vars', {'theta1', 'theta2', 'theta3'});

% Robots Dynamics Matrices
syms q1 q2 q3 qd1 qd2 qd3 real
q_sym = [q1; q2; q3];
qd_sym = [qd1; qd2; qd3];

M_sym = robot.inertia(q_sym');
C_sym = robot.coriolis(q_sym', qd_sym');
G_sym = robot.gravload(q_sym');

% Convert symbolic expressions to MATLAB functions
M_func = matlabFunction(M_sym, 'Vars', {q1, q2, q3});
C_func = matlabFunction(C_sym, 'Vars', {q1, q2, q3, qd1, qd2, qd3});
G_func = matlabFunction(G_sym, 'Vars', {q1, q2, q3});

% PSO Parameters
Particle_size = 50;
Iterations = 300;
Dimensions = 6;
c1 = 1.5;
c2 = 0.75;
w = 0.9;

% Range of values for Kp and Kd
Kp_range  =[0 1000];
Kd_range  =[0 100];

%Initializing Population
Particles_Population = zeros(Particle_size, Dimensions);
Particles_Velocity = zeros(Particle_size, Dimensions);

% Randomly populate with number between respective ranges of Kp and Kd
Particles_Population(:,1:3) = Kp_range(1) - (Kp_range(1)-Kp_range(2)).*rand(Particle_size, Dimensions/2);
Particles_Population(:,4:6) = Kd_range(1) - (Kd_range(1)-Kd_range(2)).*rand(Particle_size, Dimensions/2);
Particles_Velocity = rand(size(Particles_Velocity));

% Time span for simulation
t_span = [0 10];
initial_state = [30*pi/180; -30*pi/180; 80*pi/180; 0; 0; 0]; % Initial joint angles and velocities
initial_state_all = repmat(initial_state, 1, Particle_size);

% Initial Population Fitness
[t, state] = ode15s(@(t, state) Robot_dynamics(t, state, q_d_func, qd_d_func, Particles_Population, M_func, C_func, G_func), t_span, initial_state_all(:));
state = reshape(state, [], 6, Particle_size);
Fitness = zeros(Particle_size,1);
desired_EOF_pos = [x_func(t) y_func(t) z_func(t)];
for i = 1:Particle_size
    actual_end_effector = transform(state(:,1,i), state(:,2,i), state(:,3,i));
    Fitness(i) = mean(sqrt(sum((desired_EOF_pos - actual_end_effector).^2,2)));
end

% Initializing Global and Particle for Initial Population
PBest_Fitness = Fitness;
PBest = Particles_Population;
[GBest_Fitness, Loc] = min(Fitness);
GBest = Particles_Population(Loc,:);

GBest_history = zeros(26, 1);
PBest_avg_history = zeros(26, 1);

% Store initial values
GBest_history(1) = GBest_Fitness;
PBest_avg_history(1) = mean(PBest_Fitness);

tic
hold on
for iter = 1:300
    % Next Generation Iterations
    % Create the next Population by computing the Velocity
    Particles_Velocity = w * Particles_Velocity + c1 * rand(size(Particles_Velocity)) .* (PBest - Particles_Population) + c2 * rand(size(Particles_Velocity)) .* (GBest - Particles_Population);
    Particles_Population = Particles_Population + Particles_Velocity;
    % Ensure Kp and Kd values are positive
    Particles_Population = max(Particles_Population, 5*rand);

    [t, state] = ode15s(@(t, state) Robot_dynamics(t, state, q_d_func, qd_d_func, Particles_Population, M_func, C_func, G_func), t_span, initial_state_all(:));
    state = reshape(state, [], 6, Particle_size);
    Fitness = zeros(Particle_size,1);
    desired_EOF_pos = [x_func(t) y_func(t) z_func(t)];
    for i = 1:Particle_size
        actual_end_effector = transform(state(:,1,i), state(:,2,i), state(:,3,i));
        Fitness(i) = mean(sqrt(sum((desired_EOF_pos - actual_end_effector).^2,2)));
    end

    PBest(PBest_Fitness > Fitness, :) = Particles_Population(PBest_Fitness > Fitness, :);
    PBest_Fitness(PBest_Fitness > Fitness) = Fitness(PBest_Fitness > Fitness);
    [temp, Loc] = min(Fitness);
    if temp < GBest_Fitness
        GBest = Particles_Population(Loc,:);
        GBest_Fitness = temp;
    end
    GBest_history(iter+1) = GBest_Fitness;
    PBest_avg_history(iter+1) = mean(PBest_Fitness);
end
time = toc;

num_iterations = 300;
figure;
plot(0:num_iterations, GBest_history, 'b-', 'LineWidth', 2);
hold on;
plot(0:num_iterations, PBest_avg_history, 'r--', 'LineWidth', 1.5);
xlabel('Iteration');
ylabel('RMS Error');
title('PSO Convergence for PD Controller Optimization');
legend('Global Best Fitness', 'Particles Mean Fitness');
grid on;
