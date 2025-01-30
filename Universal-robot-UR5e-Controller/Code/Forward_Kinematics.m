function Transform_matrix = Forward_Kinematics(params)
    syms theta1 theta2 theta3 theta4 theta5 theta6
    theta = [theta1; theta2; theta3; theta4; theta5; theta6];
    
    Transform_matrix = eye(4);
    
    for i = 1:6
        % Current transformation matrix
        T_i = dh_transformation_matrix(theta(i), params.dh(i,:));
        Transform_matrix = Transform_matrix * T_i;
    end
end
