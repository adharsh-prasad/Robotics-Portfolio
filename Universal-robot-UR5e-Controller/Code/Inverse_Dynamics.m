function jointAngles = Inverse_Dynamics(position, robot, params)
    % Create an inverse kinematics solver
    ikSolver = inverseKinematics('RigidBodyTree', robot);

    % Set weights for position and orientation (favor position)
    weights = params.masses;

    % Initial guess (home configuration)
    initialGuess = homeConfiguration(robot);

    % Define desired end-effector pose as a transformation matrix
    T_desired = trvec2tform(position); 

    % Solve inverse kinematics
    [jointAngles, ~] = ikSolver('tool0', T_desired, weights, initialGuess, 'ConstraintInputs', {'position'});
end
