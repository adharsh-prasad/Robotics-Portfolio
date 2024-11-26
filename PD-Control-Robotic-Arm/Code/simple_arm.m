% Define link lengths and masses
a1 = 0.45; % Length of link 1 (meters)
m1 = 1;    % Mass of link 1 (kg)
r1 = 0.016; % Center of mass distance for link 1 (meters)

a2 = 0.45; % Length of link 2 (meters)
m2 = 1;    % Mass of link 2 (kg)
r2 = 0.016; % Center of mass distance for link 2 (meters)

% Define the links using DH parameters
L(1) = Link('d', 0, 'a', a1, 'alpha', 0);
L(1).m = m1;               % Mass of link 1
L(1).r = [0 0 a1/2];       % Center of mass for link 1
L(1).I = [m1*r1^2/2 m1*a1^2/3 m1*a1^2/3]; % Inertia tensor for link 1

L(2) = Link('d', 0, 'a', a2, 'alpha', 0);
L(2).m = m2;               % Mass of link 2
L(2).r = [a2/2 0 0];       % Center of mass for link 2
L(2).I = [m2*r2^2/2 m2*a2^2/3 m2*a2^2/3]; % Inertia tensor for link 2

% Create the robot model
robot_simple = SerialLink(L, 'name', 'Simple Robot');

% Define joint configuration (example: both joints at zero position)
q_simple = [0; 0]'; 

% Calculate the inertia matrix at the given joint configuration
try
    M_simple = robot_simple.inertia(q_simple);
    disp('Inertia matrix calculated successfully:');
    disp(M_simple);
catch e
    disp('Error calculating inertia matrix:');
    disp(e.message);
end