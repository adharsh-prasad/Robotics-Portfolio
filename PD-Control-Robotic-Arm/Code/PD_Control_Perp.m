% Define robot parameters
a1 = 45/100; m1 = 1; r1 = 1.6/100;
a2 = 45/100; m2 = 1; r2 = 1.6/100;
a3 = 45/100; m3 = 1; r3 = 1.6/100;

% Create robot model
L(1) = Link('d', a1, 'a', 0, 'alpha', -pi/2);
L(1).m = m1;
L(1).r = [0 0 a1/2];
L(1).I = [m1*a1^2/3 m1*a1^2/3 m1*r1^2/2];

L(2) = Link('d', 0, 'a', a2, 'alpha', 0);
L(2).m = m2;
L(2).r = [a2/2 0 0];
L(2).I = [m2*r2^2/2  m2*a2^2/3 m2*a2^2/3];

L(3) = Link('d', 0, 'a', a3, 'alpha', 0);
L(3).m = m3;
L(3).r = [a3/2 0 0];
L(3).I = [m3*r3^2/2  m3*a3^2/3 m3*a3^2/3];

Weld_Robot = SerialLink(L, 'name', 'Welding Arm');
Weld_Robot.gravity = [0 0 -9.81];

% PD Controller parameters
Kp = diag([645.477, 645.477, 645.477]);
Kd = diag([43.905, 43.905, 43.905]);

% Time vector
t = 0:0.01:10;

% Desired trajectory (example)
qd = [sin(t); cos(t); sin(2*t)];
dqd = [cos(t); -sin(t); 2*cos(2*t)];
ddqd = [-sin(t); -cos(t); -4*sin(2*t)];

% Initialize arrays
q = zeros(3, length(t));
dq = zeros(3, length(t));
ddq = zeros(3, length(t));
tau = zeros(3, length(t));

% Initial conditions
q(:,1) = [0.5; 0; 0];
dq(:,1) = [0; 0; 0];

% Main control loop
for i = 1:length(t)
    % Error and error derivative
    e = qd(:,i) - q(:,i);
    de = dqd(:,i) - dq(:,i);

    % PD control law
    u = Kp*e + Kd*de;

    % Get M, C, and G matrices
    M = Weld_Robot.inertia(q(:,i)');
    C = Weld_Robot.coriolis(q(:,i)', dq(:,i)');
    G = Weld_Robot.gravload(q(:,i)')';

    % Calculate required torque
    tau(:,i) = M*(ddqd(:,i) + u) + C*dqd(:,i) + G;

    % Update joint angles and velocities (simple Euler integration)
    if i < length(t)
        ddq(:,i) = M \ (tau(:,i) - C*dq(:,i) - G);
        dq(:,i+1) = dq(:,i) + ddq(:,i) * (t(i+1) - t(i));
        q(:,i+1) = q(:,i) + dq(:,i) * (t(i+1) - t(i));
    end
end

Plot results
figure;
subplot(3,1,1);
plot(t, tau(1,:));
ylabel('Torque 1 (N·m)');
xlim([1 10])
subplot(3,1,2);
plot(t, tau(2,:));
ylabel('Torque 2 (N·m)');
xlim([1 10])
subplot(3,1,3);
plot(t, tau(3,:));
ylabel('Torque 3 (N·m)');
xlabel('Time (s)');
xlim([1 10])

figure;
subplot(3,1,1);
plot(t, q(1,:), t, qd(1,:));
ylabel('Joint 1 (rad)');
legend('Actual', 'Desired');
subplot(3,1,2);
plot(t, q(2,:), t, qd(2,:));
ylabel('Joint 2 (rad)');
subplot(3,1,3);
plot(t, q(3,:), t, qd(3,:));
ylabel('Joint 3 (rad)');
xlabel('Time (s)');