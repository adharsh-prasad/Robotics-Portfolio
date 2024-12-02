function dstate = Robot_dynamics(t, state, robot, q_d_func, qd_d_func, qdd_d_func, PD_Particle)
    q = state(1:3);
    qd = state(4:6);

    % Compute desired trajectory
    q_d = q_d_func(t);
    qd_d = qd_d_func(t);
    qdd_d = qdd_d_func(t);

    % Compute control input (PD control)
    Kp = diag(PD_Particle(1,1:3)); % Proportional gains
    Kd = diag(PD_Particle(1,4:6));    % Derivative gains
    e = q_d - q;
    ed = qd_d - qd;
    tau = Kp*e + Kd*ed;
    
    M = robot.inertia(q');
    C = robot.coriolis(q', qd');
    G = robot.gravload(q');
    qdd = M\(tau - C*qd - G');
    
    % Return state derivative
    dstate = [qd; qdd];
end