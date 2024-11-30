[robot, arm_length] = Robotic_arm_model();

x_func = @(t) (35 + 25*cos(2*pi*t/10))/100;
y_func = @(t) (35 + 25*sin(2*pi*t/10))/100;
z_func = @(t) 90/100*t.^0;

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