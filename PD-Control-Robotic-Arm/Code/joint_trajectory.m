

% Main script
t_span = [0 10]; % Time span for simulation
initial_state = [0; 0; 0; 0; 0; 0]; % Initial joint angles and velocities

% ODE45 call
[t, state] = ode45(@(t, state) arm_dynamics(t, state, Weld_Robot), t_span, initial_state);

% Function to compute desired trajectory
function [q_d, qd_d, qdd_d] = desired_trajectory(time)
    syms t
    x = (25 + 10*cos(2*pi*t/10))/100;
    y = (25 + 10*sin(2*pi*t/10))/100;
    z = 70/100;
    q_d = Inverse_Dynamics([x y z], 45/100, 45/100, 45/100);
    qd_d = diff(q_d);
    qdd_d = diff(qd_d);

    % Convert symbolic to numeric
    t = time;
    q_d = double(subs(q_d));
    qd_d = double(subs(qd_d));
    qdd_d = double(subs(qdd_d));
end

% Function defining system dynamics
function dstate = arm_dynamics(t, state, robot)
    q = state(1:3);
    qd = state(4:6);

    % Compute desired trajectory
    [q_d, qd_d, qdd_d] = desired_trajectory(t);

    % Compute control input (PD control)
    Kp = diag([100, 100, 100]); % Proportional gains
    Kd = diag([10, 10, 10]);    % Derivative gains
    e = q_d - q;
    ed = qd_d - qd;
    tau = Kp*e + Kd*ed;
    disp(q)
    disp(t)
    M = robot.inertia(q');    % Mass/inertia matrix
    C = robot.coriolis(q', qd');  % Coriolis matrix
    G = robot.gravload(q');
    qdd = M\(tau - C*qd - G');
    % Return state derivative
    dstate = [qd; qdd];
end