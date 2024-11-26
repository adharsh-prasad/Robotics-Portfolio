% Main script
syms t real
x = (25 + 10*cos(2*pi*t/10))/100;
y = (25 + 10*sin(2*pi*t/10))/100;
z = 70/100;

% Compute symbolic inverse kinematics once
q_d_sym = Inverse_Dynamics([x y z], 45/100, 45/100, 45/100);
qd_d_sym = diff(q_d_sym);
qdd_d_sym = diff(qd_d_sym);

% Create MATLAB functions from symbolic expressions
q_d_func = matlabFunction(q_d_sym, 'Vars', t);
qd_d_func = matlabFunction(qd_d_sym, 'Vars', t);
qdd_d_func = matlabFunction(qdd_d_sym, 'Vars', t);

t_span = [0 10]; % Time span for simulation
initial_state = [0; 0; 0; 0; 0; 0]; % Initial joint angles and velocities

% ODE45 call
[t, state] = ode45(@(t, state) arm_dynamics(t, state, Weld_Robot, q_d_func, qd_d_func, qdd_d_func), t_span, initial_state);

% Function defining system dynamics
function dstate = arm_dynamics(t, state, robot, q_d_func, qd_d_func, qdd_d_func)
    q = state(1:3);
    qd = state(4:6);

    % Compute desired trajectory
    q_d = q_d_func(t);
    qd_d = qd_d_func(t);
    qdd_d = qdd_d_func(t);

    % Compute control input (PD control)
    Kp = diag([100, 100, 100]); % Proportional gains
    Kd = diag([10, 10, 10]);    % Derivative gains
    e = q_d - q;
    ed = qd_d - qd;
    tau = Kp*e + Kd*ed;
    
    M = robot.inertia(q');    % Mass/inertia matrix
    C = robot.coriolis(q', qd');  % Coriolis matrix
    G = robot.gravload(q');
    qdd = M\(tau - C*qd - G');
    
    % Return state derivative
    dstate = [qd; qdd];
end