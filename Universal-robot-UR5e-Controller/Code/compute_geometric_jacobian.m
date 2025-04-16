function Geometric_Jacobian = compute_geometric_jacobian(q, params)
    % Initialize Jacobian matrices
    Jv = zeros(3, 6); % Linear velocity Jacobian
    Jw = zeros(3, 6); % Angular velocity Jacobian
    
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
        
        % Compute Jacobians
        for j = 1:i
            Jv(:,j) = cross(z(:,j), p(:,i+1) - p(:,j));
            Jw(:,j) = z(:,j);
        end
    end
    
    % Combine into full Jacobian
    Geometric_Jacobian = [Jv; Jw];
end
