function dstate = Robot_dynamics(t, state, q_d_func, qd_d_func, PD_Particle, M_Matrix, C_Matrix, G_Matrix)
    num_particles = size(PD_Particle, 1);
    state = reshape(state, 6, num_particles);
    
    % Compute desired trajectory (same for all particles)
    q_d = q_d_func(t);
    qd_d = qd_d_func(t);

    % Vectorized operations
    q = state(1:3, :);
    qd = state(4:6, :);
    e = q_d - q;
    ed = qd_d - qd;

    % Pre-allocate arrays
    qdd = zeros(3, num_particles);

    % Vectorized PD control
    Kp = reshape(PD_Particle(:,1:3)', 3, num_particles);
    Kd = reshape(PD_Particle(:,4:6)', 3, num_particles);
    tau = Kp .* e + Kd .* ed;

    % Compute dynamics (this part might be challenging to fully vectorize)
    for i = 1:num_particles
        M = M_Matrix(q(1,i), q(2,i), q(3,i));
        C = C_Matrix(q(1,i), q(2,i), q(3,i), qd(1,i), qd(2,i), qd(3,i));
        G = G_Matrix(q(1,i), q(2,i), q(3,i));
        qdd(:,i) = M \ (tau(:,i) - C*qd(:,i) - G');
    end

    dstate = [qd; qdd];
    dstate = dstate(:);
end