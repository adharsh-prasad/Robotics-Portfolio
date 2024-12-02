%Initialize the robotic arm
[robot, arm_length] = Robotic_arm_model();

%Desired Trajectory
x_func = @(t) (35 + 25*cos(2*pi*t/10))/100;
y_func = @(t) (35 + 25*sin(2*pi*t/10))/100;
z_func = @(t) 90/100*t.^0;

syms t
q_d_sym = Inverse_Dynamics([x_func(t); y_func(t); z_func(t)], arm_length(1), arm_length(2), arm_length(3));
qd_d_sym = diff(q_d_sym);

% Convert symbolic expressions to MATLAB functions
q_d_func = matlabFunction(q_d_sym, 'Vars', t);
qd_d_func = matlabFunction(qd_d_sym, 'Vars', t);

t_span = [0 10]; % Time span for simulation
initial_state = [30*pi/180; -30*pi/180; 80*pi/180; 0; 0; 0]; % Initial joint angles and velocities

%Transformation Matrix
transform = Forward_Kinematics([arm_length(1) pi/2 0], [0 0 arm_length(2)], [0 0 arm_length(3)]);

%Algorithm Parameters
Particle_size = 50;
Iterations = 300;
Dimensions = 6;
c1 = 1.5;
c2 = 0.75;
w = 0.9;

%Range of values for Kp and Kd
Kp_range  =[0 1000];
Kd_range  =[0 100];

%Initializing Population
Particles_Population = zeros(Particle_size, Dimensions);
Particles_Velocity = zeros(Particle_size, Dimensions);

%Randomly populate the population with number between respective ranges of Kp and Kd
Particles_Population(:,1:3) = Kp_range(1) - (Kp_range(1)-Kp_range(2)).*rand(Particle_size, Dimensions/2);
Particles_Population(:,4:6) = Kd_range(1) - (Kd_range(1)-Kd_range(2)).*rand(Particle_size, Dimensions/2);
Particles_Velocity = rand(size(Particles_Velocity));
tic
fitness = Fitness_Function(x_func, y_func, z_func, robot, arm_length, Particles_Population(1,:), transform(1:3,4));
time = toc;

function [fitness] = Fitness_Function(x_func, y_func, z_func, robot, arm_length, PD_Particle, transform)
    % ODE45 call
    [t, state] = ode15s(@(t, state) Robot_dynamics(t, state, robot, q_d_func, qd_d_func, PD_Particle), t_span, initial_state);
    actual_end_effector = zeros(length(t),3);
    for i = 1:length(state)
        joint = state(i,1:3);
        theta1 = joint(1);
        theta2 = joint(2);
        theta3 = joint(3);
        actual_end_effector(i,:) = double(subs(transform))';
    end
    desired_EOF_pos = [x_func(t) y_func(t) z_func(t)];
    fitness = mean(sqrt(sum((desired_EOF_pos - actual_end_effector).^2,2)));
end