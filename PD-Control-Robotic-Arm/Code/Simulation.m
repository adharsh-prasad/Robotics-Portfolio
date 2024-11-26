a1 = 45/100; m1 = 1; r1 = 1.6/100;
a2 = 45/100; m2 = 1; r2 = 1.6/100;
a3 = 45/100; m3 = 1; r3 = 1.6/100;
L(1) = Link('d', a1, 'a', 0, 'alpha', -pi/2);
L(1).m = m1;
L(1).r = [0 0 a1/2];
L(1).I = [m1*a1^2/3 m1*a1^2/3 m1*r1^2/2];

L(2) = Link('d', 0, 'a', a2, 'alpha', 0);
L(2).m = m2;
L(2).r = [a2/2 0 0];
L(2).I = [m2*r2^2/2  m2*a2^2/3 m2*a2^2/3];

L(3) = Link('d', 0, 'a', a3, 'alpha', 0);
L(3).m = m3;
L(3).r = [a3/2 0 0];
L(3).I = [m3*r3^2/2  m3*a3^2/3 m3*a3^2/3];

Weld_Robot = SerialLink(L, 'name', 'Welding Arm');
Weld_Robot.gravity = [0 0 -9.81];

End_Effector_Pos = PD_Control();
joint_angles = Inverse_Dynamics(End_Effector_Pos, a1, a2, a3);

Forward_Kinematics([a1 -pi/2 0], [0 0 a2], [0 0 a3])
q = [0.1 0.1 0.1];  % Example joint configuration
M = Weld_Robot.inertia(10*q);

figure;
Weld_Robot.plot([0 0 0]);

% 
% [~,~,desired_trajectory] = PD_Control();
% joint_angles = Inverse_Kinematics();
% 
% figure(1)
% scatter3(desired_trajectory(1,:),desired_trajectory(2,:),desired_trajectory(3,:),'.r')
% hold on
% 
% for i = joint_angles
%    view([1 -1 1])
%    axis([-100 100 -100 100 -100 100]);
%    teach(Weld_Robot,i'); 
% end