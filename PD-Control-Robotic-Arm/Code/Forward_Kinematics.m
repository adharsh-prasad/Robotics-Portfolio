% FORWARD_KINEMATICS - Computes forward kinematics for the 3-DOF robotic arm
%
% Syntax: T = Forward_Kinematics(Link1, Link2, Link3)
%
% Inputs:
%    Link1 - DH parameters for first link [theta d alpha a]
%    Link2 - DH parameters for second link [theta d alpha a]
%    Link3 - DH parameters for third link [theta d alpha a]
%
% Outputs:
%    T - 4x4 homogeneous transformation matrix representing end-effector pose
%
% Example:
%    T = Forward_Kinematics([0 0.45 pi/2 0], [0 0 0 0.45], [0 0 0 0.45])
%
% See also: tranformation_matrix
function T = Forward_Kinematics(Link1, Link2, Link3)
    syms theta1 theta2 theta3
    T1 = tranformation_matrix([theta1 Link1]);
    T2 = tranformation_matrix([theta2 Link2]);
    T3 = tranformation_matrix([theta3 Link3]);
    T = T1 * T2 * T3;
end

% TRANFORMATION_MATRIX - Computes the homogeneous transformation matrix using DH parameters
%
% Syntax: T = tranformation_matrix(Link)
%
% Inputs:
%    Link - Vector containing DH parameters [theta d alpha a]
%           theta: joint angle (rad)
%           d: link offset (m)
%           alpha: link twist (rad)
%           a: link length (m)
%
% Outputs:
%    T - 4x4 homogeneous transformation matrix
%
% Example:
%    T = tranformation_matrix([pi/2 0.45 pi/2 0])
function T = tranformation_matrix(Link)
    theta = Link(1);
    d = Link(2);
    alpha = Link(3);
    a = Link(4);
    % Compute transformation matrix using DH convention
    T = [cos(theta) -sin(theta)*cos(alpha) -sin(theta)*sin(alpha) a*cos(theta);
         sin(theta)  cos(theta)*cos(alpha)  cos(theta)            a*sin(theta);
         0           sin(alpha)             cos(alpha)            d;
         0           0                      0                     1];
end
