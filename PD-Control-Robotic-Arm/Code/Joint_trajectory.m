% JOINT_TRAJECTORY - Generates joint trajectories for robotic arm motion
%
% Syntax: [t, state] = Joint_trajectory(x_func, y_func, z_func, robot, arm_length)
%
% Inputs:
%    x_func - Function handle for x-coordinate trajectory
%    y_func - Function handle for y-coordinate trajectory
%    z_func - Function handle for z-coordinate trajectory
%    robot - SerialLink object containing robot parameters
%    arm_length - Vector of link lengths [a1; a2; a3] in meters
%
% Outputs:
%    t - Time vector for trajectory points
%    state - Matrix containing joint angles and velocities over time
%           [theta1, theta2, theta3, theta1_dot, theta2_dot, theta3_dot]
%
% Example:
%    x_func = @(t) 0.35 + 0.25*cos(2*pi*t/10);
%    y_func = @(t) 0.35 + 0.25*sin(2*pi*t/10);
%    z_func = @(t) 0.9;
%    [t, state] = Joint_trajectory(x_func, y_func, z_func, robot, [0.45; 0.45; 0.45])
%
% See also: Robot_dynamics, Inverse_Dynamics
function [t, state] = Joint_trajectory(x_func, y_func, z_func, robot, arm_length)
    % Define symbolic variable for time
    syms t
    
    % Generate desired joint trajectories using inverse kinematics
    q_d_sym = Inverse_Dynamics([x_func(t); y_func(t); z_func(t)], arm_length(1), arm_length(2), arm_length(3));
    
    % Compute velocity and acceleration profiles
    qd_d_sym = diff(q_d_sym);    % First derivative (velocity)
    
    % Convert symbolic expressions to MATLAB functions
    q_d_func = matlabFunction(q_d_sym, 'Vars', t);
    qd_d_func = matlabFunction(qd_d_sym, 'Vars', t);
    
    % Define simulation parameters
    t_span = [0 10];  % Time span for simulation
    initial_state = [30*pi/180; -30*pi/180; 80*pi/180; 0; 0; 0];  % Initial joint angles and velocities
    
    % Solve equations of motion using ODE45
    [t, state] = ode45(@(t, state) Robot_dynamics(t, state, robot, q_d_func, qd_d_func), t_span, initial_state);
end
