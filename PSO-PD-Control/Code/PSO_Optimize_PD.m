%Initialize the robotic arm
[robot, arm_length] = Robotic_arm_model();

%Desired Trajectory
x_func = @(t) (35 + 25*cos(2*pi*t/10))/100;
y_func = @(t) (35 + 25*sin(2*pi*t/10))/100;
z_func = @(t) 90/100*t.^0;

%Transformation Matrix
tranform = Forward_Kinematics([arm_length(1) pi/2 0], [0 0 arm_length(2)], [0 0 arm_length(3)]);

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
Particles_Population(:,1:3) = Kp_range(1) + (Kp_range(1)-Kp_range(2)).*rand(Particle_size, Dimensions/2);
Particles_Population(:,4:6) = Kd_range(1) + (Kd_range(1)-Kd_range(2)).*rand(Particle_size, Dimensions/2);
Particles_Velocity = rand(size(Particles_Velocity));

end_effector = [];
for i = 1:length(joint_angles)
    joint = joint_angles(i,1:3);
    theta1 = joint(1);
    theta2 = joint(2);
    theta3 = joint(3);
    end_effector = [end_effector;double(subs(tranform(1:3,4)))'];
end
fitness = Joint_trajectory(x_func, y_func, z_func, robot, arm_length, PD_Particle);
