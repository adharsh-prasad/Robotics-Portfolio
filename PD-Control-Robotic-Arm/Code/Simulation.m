[robot, arm_length] = Robotic_arm_model();

x_func = @(t) (25 + 10*cos(2*pi*t/10))/100;
y_func = @(t) (25 + 10*sin(2*pi*t/10))/100;
z_func = @(t) 70/100*t.^0;

[T, joint_angles] = Joint_trajectory(x_func, y_func, z_func, robot, arm_length);
tranform = Forward_Kinematics([arm_length(1) pi/2 0], [0 0 arm_length(2)], [0 0 arm_length(3)]);
end_effector = [];
for i = 1:length(joint_angles)
    joint = joint_angles(i,1:3);
    theta1 = joint(1);
    theta2 = joint(2);
    theta3 = joint(3);
    end_effector = [end_effector;double(subs(tranform(1:3,4)))'];
end

plot3(x_func(T), y_func(T), z_func(T), 'LineWidth', 2);
hold on
scatter3(end_effector(1,1), end_effector(1,2), end_effector(1,3))
for i = 2:length(joint_angles)
    scatter3(end_effector(i,1), end_effector(i,2), end_effector(i,3), 'red')
end
xlim([0 1])
ylim([0 1])
zlim([0 1])
% figure;
% robot.plot([0 0 0]);
% 
% for i = 1:length(joint_angles)
%    teach(robot,180/pi*joint_angles(i,1:3)); 
% end