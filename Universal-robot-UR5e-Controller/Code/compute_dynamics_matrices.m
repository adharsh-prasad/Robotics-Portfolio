function [M, G] = compute_dynamics_matrices(q, params)   
    % Initialize matrices
    M = zeros(6, 6);
    G = zeros(6, 1);

    % Initialize cumulative transforms and vectors
    T = eye(4);
    z = zeros(3, 7);
    z(:,1) = [0; 0; 1];
    p = zeros(3, 7);

    % Forward pass: compute transforms and positions
    for i = 1:6
        T = T * dh_transformation_matrix(q(i), params.dh(i,:));
        z(:,i+1) = T(1:3, 3);
        p(:,i+1) = T(1:3, 4);
        
        p_c_i = p(:,i+1) + T(1:3, 1:3) * params.com(i,:)';
        
        % Compute Jacobians
        Jv_i = zeros(3, 6);
        Jw_i = zeros(3, 6);
        for j = 1:i
            Jv_i(:,j) = cross(z(:,j), p_c_i - p(:,j));
            Jw_i(:,j) = z(:,j);
        end
        
        % Compute inertia in world frame
        I_i = T(1:3, 1:3) * params.inertia{i} * T(1:3, 1:3)';
        
        % Update M matrix
        M = M + params.masses(i) * (Jv_i' * Jv_i) + Jw_i' * I_i * Jw_i;
        
        % Update G vector
        G = G + params.masses(i) * Jv_i' * [0;0;-9.81];
    end
end