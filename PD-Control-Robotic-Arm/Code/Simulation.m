% [robot, arm_length] = Robotic_arm_model();
% 
% x_func = @(t) (25 + 10*cos(2*pi*t/10))/100;
% y_func = @(t) (25 + 10*sin(2*pi*t/10))/100;
% z_func = @(t) 70/100;
% 
% [T, joint_angles] = Joint_trajectory(x_func, y_func, z_func, robot, arm_length);
figure;
robot.plot([0 0 0]);

for i = 1:length(joint_angles)
   teach(robot,360/pi*joint_angles(i,1:3)); 
end