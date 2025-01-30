function robot = create_ur5e_model()
    params = Robotic_arm_model();
    % UR5e DH parameters
    
    a = params.dh(:,2);      % Link lengths
    d =  params.dh(:,3); % Link offsets
    alpha = params.dh(:,4);            % Link twists
    
    % Create links
    for i = 1:6
        L(i) = Link('d', d(i), 'a', a(i), 'alpha', alpha(i));
        L(i).m = params.masses(i);
        L(i).r = params.com(i,:);
        L(i).I = params.inertia{i};
    end
    
    % Create SerialLink robot object
    robot = SerialLink(L, 'name', 'UR5e');
    robot.gravity = [0 0 -9.81];
end
