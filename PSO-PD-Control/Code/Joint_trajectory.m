function [t, state] = Joint_trajectory(x_func, y_func, z_func, robot, arm_length, PD_Particle)
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
    [t, state] = ode45(@(t, state) Robot_dynamics(t, state, robot, q_d_func, qd_d_func, qdd_d_func), t_span, initial_state, PD_Particle);
end