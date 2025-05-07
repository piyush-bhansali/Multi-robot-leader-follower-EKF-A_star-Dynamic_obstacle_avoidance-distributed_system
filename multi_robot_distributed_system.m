clc;
clear;
close all;

%Robot parameters
robot = struct('robotRadius', 0.2, 'wheelRadius', 0.1, 'axleLength', 0.3, 'maxSpeed', 0.5, 'maxOmega', 0.1, 'numOfTicks', 1000, 'lookAheadDistance', 1.5, 'safeDistance', 2, 'numberOfFollower', 2);

%environment parameters
environment = struct('gridSize', 1, 'envSize', [25, 25], 'numGrids', [25, 25], 'numObstacles', 3, 'maxIters', 400, 'start', [1, 1], 'goal', [22, 22], 'goalThreshold', 0.5);

%path planning
pathProperties = struct('nextNodeDist', 1, 'numPoints', 10, 'safeDistance', 1.5);

simulationTime = 800;
dt = 0.1;

environment.numGrids = environment.envSize ./ environment.gridSize;
robot.maxOmega = robot.maxSpeed/robot.axleLength;


%%

% Active Leader Flag
activeLeader = true;
followerLost = false;

% Create a matrix to represent the grid and initialize all grids with 0
global astar_path;
global newPath;
global gridValues;
gridValues = zeros(environment.numGrids);

% Generate random obstacles
obstacles = cell(environment.numObstacles, 1);
rng('shuffle');
for i = 1:environment.numObstacles
    % Randomly set position and size of the obstacles
    obsX = 5  + (environment.envSize(1) - 10) * rand(); % limit range for better visualization
    obsY = 5  + (environment.envSize(1) - 10) * rand();
    obsRadius = 2 + 0.5 * rand();

    % Store obstacle as a struct
    obstacles{i} = struct('x', obsX, 'y', obsY, 'radius', obsRadius);
end

%%

% Initial position
leaderPose = [2; 2; pi/4];
environment.start = leaderPose(1:2);

N = robot.numberOfFollower;
followerPose = zeros(3, N);

for i = 1 : N
    followerPose(:, i) = [i+1; 1; pi/2];
end
%%

% EKF Initialization
processNoise = 0.01^2 * eye(2);
gpsNoise = 0.5^2 * eye(2); % GPS noise (covariance matrix)
lidarNoise = 0.01^2 * eye(2);

% Leader EKF
leaderEkf.state = leaderPose;
leaderEkf.P = eye(3);
leaderEkf.Q = processNoise;
leaderEkf.R_Lidar = lidarNoise;

% Follower EKF (inherits same parameters as leader)
for i = 1 : N
    followerEkf(i).state = followerPose(:,i);
    followerEkf(i).P = eye(3);
    followerEkf(i).Q = processNoise;
    followerEkf(i).R_GPS = gpsNoise;
end

%%

% Data Storage for Plotting


leaderTrajectory = zeros(0, 2);
leaderTrueTrajectory = zeros(0, 2);
followerTrajectory     = cell(N, 1);
followerTrueTrajectory = cell(N, 1);
for i = 1 : N
    followerTrajectory{i} = zeros(0, 2);
    followerTrueTrajectory{i} = zeros(0, 2);
end
%%
% GPS Outage Parameters
gpsOutageProbability = 0.005;
gpsOutageTimeout = 2;
gpsOutageActive = false;

%Follower Dead Reckoning Initialization
followerLastUpdate = 0;
lastFollowerValidGPSUpdate = 0;

% Initialize velocities
leaderLinearVelocity = 0;
leaderAngularVelocity = 0;
% Preallocate follower velocities as 1×N vectors
followerLinearVelocity  = zeros(N, 1);
followerAngularVelocity = zeros(N, 1);

%%

% PID controller gains (same for all followers here, but you can make each entry different)
Kp = 1.5 * ones(N, 1);   % proportional gains
Ki = 0.2 * ones(N, 1);   % integral gains
Kd = 0.1 * ones(N, 1);   % derivative gains

% Initialize per‐follower error terms
integralError   = zeros(N, 1);  % ∫error·dt
previousError   = zeros(N, 1);  % error at t−dt

% Pack into a single struct for passing into your control function
PID = struct('Kp', Kp, 'Ki', Ki, 'Kd', Kd);

% Initialize plot update parameters
plot_counter = 0;
plot_update_interval = 2; % Number of iterations between plot updates

astar_path = astar(leaderEkf.state, environment, gridValues, pathProperties);

%%

figure;
%hold on;
while activeLeader
    for t = 0:dt:simulationTime
        newPath = false;
        distanceToFinalWaypoint = norm(leaderEkf.state(1:2) - environment.goal);
        for i = 1 : robot.numberOfFollower
            distanceToFollower(i) = norm(leaderEkf.state(1:2) - followerEkf(i).state(1:2));
        end
        if distanceToFinalWaypoint < environment.goalThreshold % Stop at the end of path
            leaderLinearVelocity = 0;
            leaderAngularVelocity = 0;
            break;

        elseif any(distanceToFollower > robot.safeDistance)
            disp("Waiting for the follower");
            leaderLinearVelocity = 0;
            leaderAngularVelocity = 0;

        else
            [leaderLinearVelocity, leaderAngularVelocity] = pure_pursuit_control(leaderEkf.state, astar_path, obstacles, gridValues, robot, environment, pathProperties, followerLost);
            state_true = kinematic_model(leaderEkf.state, leaderLinearVelocity, leaderAngularVelocity, dt);
            leaderTrueTrajectory(end+1, :) = state_true(1:2)';
            [ticks_L, ticks_R] = get_encoder_values(leaderLinearVelocity, leaderAngularVelocity, dt, robot);
            [leaderEkf.state, leaderEkf.P] = ekf_predict(leaderEkf.state, [ticks_L, ticks_R], leaderEkf.P, leaderEkf.Q, robot);
        end

        %%
        % --- Simulate LIDAR readings & EKF updates for all followers ---

        % Pre‐allocate arrays for LIDAR measurements
        d   = zeros(N, 1);   % distances to each follower
        phi = zeros(N, 1);   % bearing angles to each follower

        for i = 1:N
            % get distance & bearing from leader to follower i
            [d(i), phi(i)] = lidarMeasurement(leaderEkf.state, followerEkf(i).state);

            % pack into measurement vector
            z = [d(i); phi(i)];

            % 3) EKF update of the leader using follower i’s position
            [ leaderEkf.state, leaderEkf.P ] = ekf_update(leaderEkf.state, leaderEkf.P, z, leaderEkf.R_Lidar, followerEkf(i).state);
        end

        % --- Store leader’s updated position for plotting ---
        leaderTrajectory(end+1, :) = leaderEkf.state(1:2)';  % append [x, y] as a new row


        %---------------- Follower Robot Naviagtion-------------------

        followerLastUpdate(:) = t;
        desiredFollowerPose = leaderEkf.state;

        for i = 1:N

            % Calculate the error in distance
            currentDistance = norm(followerEkf(i).state(1:2) - desiredFollowerPose(1:2));
            errorDist(i) = abs(currentDistance - robot.safeDistance);


            % 2) Update PID integrator and derivative terms
            integralError(i)    = integralError(i) + errorDist(i)*dt;
            derivativeError(i)  = (errorDist(i) - previousError(i)) / dt;
            previousError(i)    = errorDist(i);
            error(i) = struct('integralError',   integralError(i), 'derivativeError', derivativeError(i), 'previousError', previousError(i));

            PID_i = struct('Kp', PID.Kp(i), 'Ki', PID.Ki(i), 'Kd', PID.Kd(i));

            [followerLinearVelocity(i), followerAngularVelocity(i)] = followerPurePursuit(followerEkf(i), desiredFollowerPose, error(i), PID_i, robot);

            followerStateTrue = kinematic_model(followerEkf(i).state, followerLinearVelocity(i), followerAngularVelocity(i), dt);
            followerTrueTrajectory{i}(end+1, :) = followerStateTrue(1:2)';

            [follower_ticks_L, follower_ticks_R] = get_encoder_values(followerLinearVelocity(i), followerAngularVelocity(i), dt, robot);
            [followerEkf(i).state, followerEkf(i).P] = ekf_predict_follower(followerEkf(i).state, [follower_ticks_L, follower_ticks_R], followerEkf(i).P, followerEkf(i).Q, robot);
        end

        % Simulate GPS outages & EKF updates for all followers

        for i = 1:N
            % 1) Simulate possible GPS outage for follower i
            if rand < gpsOutageProbability
                gpsMeasurement = [NaN; NaN];
                % check if outage has lasted too long
                if t - lastFollowerValidGPSUpdate(i) > gpsOutageTimeout
                    gpsOutageActive(i) = true;
                    disp(['Persistent GPS outage for follower ' num2str(i)]);
                end
            else
                % normal GPS reading with noise
                gpsMeasurement = followerEkf(i).state(1:2) + 0.1*randn(2,1);
                % clamp to map bounds [0, envSize]
                gpsMeasurement = min(max(gpsMeasurement, 0), environment.envSize(1));
                lastFollowerValidGPSUpdate(i) = t;
                gpsOutageActive(i) = false;
            end

            % 2) EKF update step for follower i
            [ followerEkf(i).state, followerEkf(i).P] = ekf_update_follower(followerEkf(i).state, followerEkf(i).P, gpsMeasurement, followerEkf(i).R_GPS);

            % 3) Store follower i’s updated position
            followerTrajectory{i}(end+1, :) = followerEkf(i).state(1:2)';
        end


        %%
        % Update the plot less frequently to reduce flickering
        plot_counter = plot_counter + 1;
        if plot_counter >= plot_update_interval
            % Plotting (Now inside a conditional statement)
            if t > 0  % Plot only after the first iteration

                % Plot obstacles
                for j = 1:environment.numObstacles
                    obs = obstacles{j};
                    rectangle('Position', [obs.x - obs.radius, obs.y - obs.radius, obs.radius * 2, obs.radius * 2], ...
                        'Curvature', [1, 1], 'FaceColor', [1, 1, 0.7]);
                end

                plot(environment.start(1), environment.start(2), 'go', 'MarkerSize', 10, 'MarkerFaceColor', 'g'); % Start
                plot(environment.goal(1), environment.goal(2), 'ro', 'MarkerSize', 10, 'MarkerFaceColor', 'r'); % Goal

                % Plot the a_star path
                plot(astar_path(:,1), astar_path(:,2), 'bo-', 'LineWidth', 1, 'MarkerSize', 3, 'MarkerFaceColor', 'b');

                % Plot leader trajectory
                plot(leaderTrajectory(:, 1), leaderTrajectory(:, 2), 'g-', 'LineWidth', 2);
                % Follower trajectories
                for j = 1:N
                    traj = followerTrajectory{j};
                    plot(traj(:,1), traj(:,2), '-', 'LineWidth', 2);
                end

                % Draw leader robot
                xL     = leaderEkf.state(1);
                yL     = leaderEkf.state(2);
                thetaL = leaderEkf.state(3);
                t_circle = linspace(0, 2*pi, 50);
                xC     = xL + robot.robotRadius * cos(t_circle);
                yC     = yL + robot.robotRadius * sin(t_circle);
                plot(xC, yC, 'g-', 'LineWidth', 2);
                % Heading line
                heading_length = 0.5;
                xE = xL + heading_length * cos(thetaL);
                yE = yL + heading_length * sin(thetaL);
                plot([xL, xE], [yL, yE], 'r-', 'LineWidth', 2);

                % Draw follower robots and headings
                for j = 1:N
                    xF     = followerEkf(j).state(1);
                    yF     = followerEkf(j).state(2);
                    thetaF = followerEkf(j).state(3);
                    xCf = xF + robot.robotRadius * cos(t_circle);
                    yCf = yF + robot.robotRadius * sin(t_circle);
                    plot(xCf, yCf, 'r-', 'LineWidth', 2);
                    xEf = xF + heading_length * cos(thetaF);
                    yEf = yF + heading_length * sin(thetaF);
                    plot([xF, xEf], [yF, yEf], 'r-', 'LineWidth', 2);
                end

                % Plot settings
                xlim([0 environment.envSize(1)]);
                ylim([0 environment.envSize(2)]);
                xlabel('X Position');
                ylabel('Y Position');
                title('Leader and Follower Robot Trajectories');
                drawnow;

                clf; % Clear the plot
                hold on; % Hold for the next iteration
            end
            plot_counter = 0;
        end

    end
    activeLeader = false;
    hold off;
end

% After the loop, plot the final estimated trajectories

% --- Estimated Trajectories ---
figure;
hold on;

% Plot obstacles
for j = 1:environment.numObstacles
    obs = obstacles{j};
    rectangle( ...
        'Position', [obs.x - obs.radius, obs.y - obs.radius, obs.radius*2, obs.radius*2], ...
        'Curvature', [1,1], ...
        'FaceColor', [1,1,0.7] ...
        );
end

% Plot leader estimate
plot(leaderTrajectory(:,1), leaderTrajectory(:,2), 'b-', 'LineWidth', 2);

% Plot each follower estimate
for i = 1:N
    traj = followerTrajectory{i};
    plot(traj(:,1), traj(:,2), '-', 'LineWidth', 2);
end

% Draw final pose of leader
t_circle = linspace(0,2*pi,50);
xL = leaderEkf.state(1);
yL = leaderEkf.state(2);
thetaL = leaderEkf.state(3);
xC = xL + robot.robotRadius*cos(t_circle);
yC = yL + robot.robotRadius*sin(t_circle);
plot(xC, yC, 'g-', 'LineWidth', 2);

% Draw final pose of each follower
for i = 1:N
    xF = followerEkf(i).state(1);
    yF = followerEkf(i).state(2);
    thetaF = followerEkf(i).state(3);
    xCF = xF + robot.robotRadius*cos(t_circle);
    yCF = yF + robot.robotRadius*sin(t_circle);
    plot(xCF, yCF, 'r-', 'LineWidth', 2);
end

% Final plot settings
xlim([0, environment.envSize(1)]);
ylim([0, environment.envSize(2)]);
xlabel('X Position');
ylabel('Y Position');
title('Final Estimated Trajectories (Leader & Followers)');
hold off;


% --- True Trajectories ---
figure;
hold on;

% Plot leader true path
plot(leaderTrueTrajectory(:,1), leaderTrueTrajectory(:,2), 'r-', 'LineWidth', 2);

% Plot each follower true path
for i = 1:N
    trueTraj = followerTrueTrajectory{i};
    plot(trueTraj(:,1), trueTraj(:,2), '-', 'LineWidth', 2);
end

% True plot settings
xlim([0, environment.envSize(1)]);
ylim([0, environment.envSize(2)]);
xlabel('X Position');
ylabel('Y Position');
title('Final True Trajectories (Leader & Followers)');
hold off;


% --- Obstacle Mapping Grid ---
figure;
hold on;
ngX = environment.numGrids(1);
ngY = environment.numGrids(2);
for ix = 1:ngX
    for iy = 1:ngY
        if gridValues(ix,iy) == 1
            rectangle( ...
                'Position', [ (ix-1)*environment.gridSize, (iy-1)*environment.gridSize, ...
                environment.gridSize, environment.gridSize ], ...
                'EdgeColor', 'k' ...
                );
            text( ix-0.5, iy-0.5, num2str(gridValues(ix,iy)), ...
                'HorizontalAlignment','center', 'VerticalAlignment','middle' );
        end
    end
end
axis equal;
xlim([0, ngX*environment.gridSize]);
ylim([0, ngY*environment.gridSize]);
xlabel('X position');
ylabel('Y position');
title('Obstacle Mapping');
hold off;


