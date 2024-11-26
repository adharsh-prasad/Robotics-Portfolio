function joint_angles = Joint_Angles()
    % Link lengths
    L1 = 45;
    L2 = 45;
    L3 = 45;
    [End_Effector_Pos,~,~] = PD_Control();
    
    % Calculate joint angles as a function of time
    joint_angles = zeros(3, length(End_Effector_Pos));
    for i = 1:length(End_Effector_Pos)
        joint_angles(:, i) = Inverse_Dynamics(End_Effector_Pos(:,i), L1, L2, L3);
    end
end