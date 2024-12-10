% INVERSE_DYNAMICS - Computes inverse kinematics for a 3-DOF robotic arm
%
% Inputs:
%    position - Target end-effector position [x; y; z] in meters
%    a1 - Length of first link (m)
%    a2 - Length of second link (m)
%    a3 - Length of third link (m)
%
% Outputs:
%    angles - Joint angles [theta1; theta2; theta3] in radians
%            theta1: base rotation angle
%            theta2: shoulder angle
%            theta3: elbow angle
%
% Example:
%    angles = Inverse_Dynamics([0.5; 0.5; 0.5], 0.45, 0.45, 0.45)
%
% Notes:
%    Uses geometric approach to solve inverse kinematics
%    Assumes elbow-up configuration
%
% See also: Forward_Kinematics
function angles = Inverse_Dynamics(position, a1, a2, a3)
    % Extract cartesian coordinates
    x = position(1);
    y = position(2);
    z = position(3);
    
    % Compute base rotation angle
    theta1 = atan2(y, x);
    
    % Compute elbow angle using cosine law
    theta3 = acos((x^2 + y^2 + (z-a1)^2 - a2^2 - a3^2) / (2 * a2 * a3));
    
    % Compute distance to end effector projection
    k = sqrt(x^2 + y^2 + (z-a1)^2);
    
    % Compute shoulder angle using atan2 and cosine law
    theta2 = atan2(z-a1, sqrt(x^2 + y^2)) - acos((a2^2 + k^2 - a3^2) / (2 * a2 * k));
    
    % Return joint angles
    angles = [theta1; theta2; theta3];
end
