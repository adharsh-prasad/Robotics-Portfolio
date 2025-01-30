% robot = create_ur5e_model();
% num_samples = 10000;
% workspace = zeros(num_samples, 3);
% 
% for i = 1:num_samples
%     q = rand(6,1) .* (robot.qlim(:,2) - robot.qlim(:,1)) + robot.qlim(:,1);
%     T = robot.fkine(q);
%     workspace(i,:) = T.t;  % This line is correct now
% end
% 
% figure;
% scatter3(workspace(:,1), workspace(:,2), workspace(:,3), 5, 'filled');
% xlabel('X (m)');
% ylabel('Y (m)');
% zlabel('Z (m)');
% title('UR5e Robot Workspace Approximation');
% axis equal;
% grid on;
[GBest, GBest_history] = PSO_Optimize_PD();
save('GBest_parameters.mat', 'GBest', 'GBest_fitness');