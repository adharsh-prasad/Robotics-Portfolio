function dstate = Robot_dynamics(t, state, q_d_traj, qd_d_traj, Kp, Kd, params, lin)
    q = state(1:6);
    qd = state(7:12);

    % dynamics matrices
    [M, G] = compute_dynamics_matrices(q, params);
    
    Cqd = compute_Cqd(q, qd, params, M);

    % Cqd_cus = robot.coriolis(q', qd');
    % disp(sum(Cqd_cus*qd - Cqd))

    % Rest of the computations
    q_d = interp1(lin, q_d_traj, t);
    qd_d = interp1(lin, qd_d_traj, t);
    
    e = q_d' - q;
    edot = qd_d' - qd;
    tau = Kp * e + Kd * edot;
       
    qdd = M \ (tau - Cqd - G);
    dstate = [qd; qdd];
end

function Cqd = compute_Cqd(q, qd, params, M0)
    delta = 1e-6;
    
    % Precompute all mass matrices upfront (7 calls instead of 648)
    M_pert = zeros(6,6,6);  % 3D array: M_pert(:,:,k) = M(q_k + δ)
    
    for k = 1:6
        q_pert = q;
        q_pert(k) = q_pert(k) + delta;
        M_pert(:,:,k) = compute_dynamics_matrices(q_pert, params);
    end

    % Compute Christoffel symbols using precomputed matrices
    C = zeros(6,6);
    for i = 1:6
        for j = 1:6
            for k = 1:6
                dMij_dqk = (M_pert(i,j,k) - M0(i,j)) / delta;
                dMik_dqj = (M_pert(i,k,j) - M0(i,k)) / delta;
                dMjk_dqi = (M_pert(j,k,i) - M0(j,k)) / delta;
                
                C(i,j) = C(i,j) + 0.5*(dMij_dqk + dMik_dqj - dMjk_dqi)*qd(k);
            end
        end
    end
    
    Cqd = C*qd;
end
