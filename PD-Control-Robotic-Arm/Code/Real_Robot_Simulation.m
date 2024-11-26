robot = loadrobot('kinovaGen3');
% Set a specific configuration
config = homeConfiguration(robot);
config(1).JointPosition = pi/4;
config(3).JointPosition = -pi/3;

% Show the robot in this configuration
figure;
show(robot, config);
title('KUKA LBR iiwa 7 Robot - Custom Configuration');
xlabel('X');
ylabel('Y');
zlabel('Z');
view(3);
grid on;