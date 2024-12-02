function [fitness] = Fitness_Function(x_func, y_func, z_func, robot, arm_length, PD_Particle, transform)
    syms t
    q_d_sym = Inverse_Dynamics([x_func(t); y_func(t); z_func(t)], arm_length(1), arm_length(2), arm_length(3));
    qd_d_sym = diff(q_d_sym);
    qdd_d_sym = diff(qd_d_sym);
    
    % Convert symbolic expressions to MATLAB functions
    q_d_func = matlabFunction(q_d_sym, 'Vars', t);
    qd_d_func = matlabFunction(qd_d_sym, 'Vars', t);
    qdd_d_func = matlabFunction(qdd_d_sym, 'Vars', t);
    
    t_span = [0 10]; % Time span for simulation
    initial_state = [30*pi/180; -30*pi/180; 80*pi/180; 0; 0; 0]; % Initial joint angles and velocities
    
    % ODE45 call
    tic
    options = odeset('RelTol', 1e-3, 'AbsTol', 1e-6);
    [t, state] = ode15s(@(t, state) Robot_dynamics(t, state, robot, q_d_func, qd_d_func, qdd_d_func, PD_Particle), t_span, initial_state);
    time = toc;
    disp(time)
    actual_end_effector = [];
    for i = 1:length(state)
        joint = state(i,1:3);
        theta1 = joint(1);
        theta2 = joint(2);
        theta3 = joint(3);
        actual_end_effector = [actual_end_effector;double(subs(transform))'];
    end
    desired_EOF_pos = [x_func(t) y_func(t) z_func(t)];
    fitness = mean(sqrt(sum((desired_EOF_pos - actual_end_effector).^2,2)));
end