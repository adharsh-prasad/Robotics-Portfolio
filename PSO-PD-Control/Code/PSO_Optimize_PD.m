function [best_Kp, best_Kd] = PSO_Optimize_PD()
    % PSO parameters
    num_particles = 50;
    num_dimensions = 6;  % Kp and Kd
    max_iterations = 300;
    c1 = 1.5;  % Cognitive coefficient
    c2 = 0.75;  % Social coefficient
    w = 0.9;  % Initial inertia weight

    % Define search space
    Kp_range = [0, 1000];
    Kd_range = [0, 100];

    % Initialize particles
    particles = zeros(num_particles, num_dimensions);
    velocities = zeros(num_particles, num_dimensions);
    for i = 1:num_particles
        for j = 1:3
            particles(i, j) = Kp_range(1) + rand() * (Kp_range(2) - Kp_range(1));
            particles(i, j) = Kd_range(1) + rand() * (Kd_range(2) - Kd_range(1));
        end
        velocities(i, :) = rand(1, num_dimensions);
    end

    % Initialize personal best and global best
    pbest = particles;
    pbest_fitness = Inf(num_particles, 1);
    [gbest_fitness, gbest_index] = min(pbest_fitness);
    gbest = pbest(gbest_index, :);

    % Main PSO loop
    for iter = 1:max_iterations
        % Update inertia weight
        w = 0.9 - 0.5 * (iter / max_iterations);

        for i = 1:num_particles
            % Evaluate fitness
            fitness = evaluate_fitness(particles(i, :));

            % Update personal best
            if fitness < pbest_fitness(i)
                pbest_fitness(i) = fitness;
                pbest(i, :) = particles(i, :);
            end

            % Update global best
            if fitness < gbest_fitness
                gbest_fitness = fitness;
                gbest = particles(i, :);
            end

            % Update velocity and position
            r1 = rand(1, num_dimensions);
            r2 = rand(1, num_dimensions);
            velocities(i, :) = w * velocities(i, :) + ...
                               c1 * r1 .* (pbest(i, :) - particles(i, :)) + ...
                               c2 * r2 .* (gbest - particles(i, :));
            particles(i, :) = particles(i, :) + velocities(i, :);

            % Enforce bounds
            particles(i, 1) = max(Kp_range(1), min(Kp_range(2), particles(i, 1)));
            particles(i, 2) = max(Kd_range(1), min(Kd_range(2), particles(i, 2)));
        end

        % Display progress
        if mod(iter, 10) == 0
            fprintf('Iteration %d: Best fitness = %.6f\n', iter, gbest_fitness);
        end
    end

    best_Kp = gbest(1);
    best_Kd = gbest(2);
end

function fitness = evaluate_fitness(params)
    Kp = params(1);
    Kd = params(2);

    % Run simulation with current Kp and Kd
    [T, joint_angles] = simulate_robotic_arm(Kp, Kd);

    % Calculate RMS error
    desired_trajectory = [sin(T), sin(T)];
    error = desired_trajectory - joint_angles;
    rms_error = sqrt(mean(error.^2, 'all'));

    fitness = rms_error;
end

function [T, joint_angles] = simulate_robotic_arm(Kp, Kd)
    % Implement your robotic arm simulation here
    % This should return the time vector T and joint angles
    % You can use the dynamics equations and PD controller from your existing code
end