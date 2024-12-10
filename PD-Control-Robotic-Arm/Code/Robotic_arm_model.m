% ROBOTIC_ARM_MODEL - Creates a 3-DOF robotic arm model with dynamic properties
%
% Outputs:
%    robot - SerialLink object containing complete robot model
%    arm_length - Vector of link lengths [a1; a2; a3] in meters
%
% Customizable Parameters:
% Link lengths (a1, a2, a3): Default 0.45m each
% Link masses (m1, m2, m3): Default 1kg each
% Link radii (r1, r2, r3): Default 0.016m each
% All parameters can be modified as per user requirements
%
% Notes:
%    Uses Peter Corke's Robotics Toolbox conventions
%    Includes mass, inertia, and gravity properties
%    Configured for vertical plane operation
%

function [robot, arm_length] = Robotic_arm_model()
    % Define physical parameters
    a1 = 45/100; m1 = 1; r1 = 1.6/100;  % First link parameters
    a2 = 45/100; m2 = 1; r2 = 1.6/100;  % Second link parameters
    a3 = 45/100; m3 = 1; r3 = 1.6/100;  % Third link parameters
    arm_length = [a1;a2;a3];
    
    % Define first link (vertical)
    L(1) = Link('d', a1, 'a', 0, 'alpha', pi/2);
    L(1).m = m1;  % Mass
    L(1).r = [0 0 a1/2];  % Center of mass
    L(1).I = [m1*a1^2/3 m1*a1^2/3 m1*r1^2/2];  % Inertia tensor

    % Define second link (horizontal)
    L(2) = Link('d', 0, 'a', a2, 'alpha', 0);
    L(2).m = m2;
    L(2).r = [a2/2 0 0];
    L(2).I = [m2*r2^2/2  m2*a2^2/3 m2*a2^2/3];

    % Define third link (horizontal)
    L(3) = Link('d', 0, 'a', a3, 'alpha', 0);
    L(3).m = m3;
    L(3).r = [a3/2 0 0];
    L(3).I = [m3*r3^2/2  m3*a3^2/3 m3*a3^2/3];

    % Create SerialLink robot object
    robot = SerialLink(L, 'name', 'Welding Arm');
    robot.gravity = [0 0 -9.81];  % Define gravity vector
end
