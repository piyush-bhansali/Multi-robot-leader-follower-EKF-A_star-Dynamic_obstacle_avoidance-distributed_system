clc;
clear;
close all;

%Robot parameters
robot = struct('robotRadius', 0.2, 'wheelRadius', 0.1, 'axleLength', 0.3, 'maxSpeed', 0.5, 'maxOmega', 0.1, 'numOfTicks', 1000, 'lookAheadDistance', 1.5, 'safeDistance', 2, 'numberOfFollower', 2);

%environment parameters
environment = struct('gridSize', 1, 'envSize', [30, 30], 'numGrids', [25, 25], 'numObstacles', 5, 'maxIters', 400, 'start', [1, 1], 'goal', [28, 28], 'goalThreshold', 0.5);

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
follower1Pose = [1; 1; pi/4];
environment.start = leaderPose(1:2);

% EKF Initialization
processNoise = 0.01^2 * eye(2); 
gpsNoise = 0.5^2 * eye(2); % GPS noise (covariance matrix)
lidarNoise = 0.01^2 * eye(2);

% Leader EKF
leaderEkf.state = leaderPose;
leaderEkf.P = eye(3);
leaderEkf.Q = processNoise;
leaderEkf.R_Lidar = lidarNoise;

% Follower 1 EKF (inherits same parameters as leader)
follower1Ekf.state = follower1Pose;
follower1Ekf.P = eye(3);
follower1Ekf.Q = processNoise;
follower1Ekf.R_GPS = gpsNoise;

% Data Storage for Plotting
leaderTrajectory = zeros(0, 2);
leaderFilteredTrajectory = zeros(0, 2);
leaderTrueTrajectory = zeros(0, 2);
follower1Trajectory = zeros(0, 2);
follower1FilteredTrajectory = zeros(0, 2);
followerTrueTrajectory = zeros(0, 2);

% GPS Outage Parameters
gpsOutageProbability = 0.005;
gpsOutageTimeout = 2;
gpsOutageActive = false;

% Dead Reckoning Initialization
lastValidGPSUpdate = 0;

%Follower Dead Reckoning Initialization
follower1LastUpdate = 0;
follower1UpdateTimeout = 5; % Timeout in seconds
lastFollowerValidGPSUpdate = 0;

% Initialize velocities
leaderLinearVelocity = 0;
leaderAngularVelocity = 0;
follower1LinearVelocity = 0;
follower1AngularVelocity = 0;

% PID controller parameters for Follower 1
Kp1 = 1.5;
Ki1 = 0.2;
Kd1 = 0.1;
integralError = 0;
previousError = 0;
PID = struct('Kp1', Kp1, 'Ki1', Ki1, 'Kd1', Kd1);

% Initialize plot update parameters
plot_counter = 0;
plot_update_interval = 2; % Number of iterations between plot updates

astar_path = astar(leaderEkf.state, environment, gridValues, pathProperties);

%% 

figure;
hold on;
i=1;
while activeLeader
    for t = 0:dt:simulationTime
        newPath = false;
        distanceToFinalWaypoint = norm(leaderEkf.state(1:2) - environment.goal);
        distanceToFollower = norm(leaderEkf.state(1:2) - follower1Ekf.state(1:2));
         if distanceToFinalWaypoint < environment.goalThreshold % Stop at the end of path
            leaderLinearVelocity = 0;
            leaderAngularVelocity = 0;
            break;

         elseif distanceToFollower > robot.safeDistance
            disp("Waiting for the follower");
            leaderLinearVelocity = 0;
            leaderAngularVelocity = 0;

         else
            [leaderLinearVelocity, leaderAngularVelocity] = pure_pursuit_control_v3(leaderEkf.state, astar_path, obstacles, gridValues, robot, environment, pathProperties, followerLost);
            state_true = kinematic_model(leaderEkf.state, leaderLinearVelocity, leaderAngularVelocity, dt);
            leaderTrueTrajectory = [leaderTrueTrajectory; state_true(1:2)'];
            [ticks_L, ticks_R] = get_encoder_values(leaderLinearVelocity, leaderAngularVelocity, dt, robot);
            [leaderEkf.state, leaderEkf.P] = ekf_predict(leaderEkf.state, [ticks_L, ticks_R], leaderEkf.P, leaderEkf.Q, robot);
         end

%%

        %Simulate LIDAR reading

        [d, phi] = lidarMeasurement(leaderEkf.state, follower1Ekf.state);

                 
        [leaderEkf.state, leaderEkf.P] = ekf_update(leaderEkf.state, leaderEkf.P, [d, phi], leaderEkf.R_Lidar, follower1Ekf.state);

        % Store Data for leader
        leaderTrajectory = [leaderTrajectory; leaderEkf.state(1:2)'];

          %---------------- Follower Robot Naviagtion-------------------

        follower1LastUpdate = t;
        desiredFollower1Pose = leaderEkf.state;

        % Calculate the error in distance
        currentDistance = norm(follower1Ekf.state(1:2) - desiredFollower1Pose(1:2));
        errorDist = abs(currentDistance - robot.safeDistance);

        % Update integral and derivative errors
        integralError = integralError + errorDist * dt;
        derivativeError = (errorDist - previousError) / dt;
        previousError = errorDist;
        error = struct('integralError', integralError, 'derivativeError', derivativeError, 'previousError', previousError);


        [follower1LinearVelocity, follower1AngularVelocity] = followerPurePursuit(follower1Ekf, desiredFollower1Pose, error, PID, robot);

        follower_state_true = kinematic_model(follower1Ekf.state, follower1LinearVelocity, follower1AngularVelocity, dt);
        followerTrueTrajectory = [followerTrueTrajectory; follower_state_true(1:2)'];
        [follower_ticks_L, follower_ticks_R] = get_encoder_values(follower1LinearVelocity, follower1AngularVelocity, dt, robot);
        [follower1Ekf.state, follower1Ekf.P] = ekf_predict_follower_v3(follower1Ekf.state, [follower_ticks_L, follower_ticks_R], follower1Ekf.P, follower1Ekf.Q, robot);

         % Simulate GPS Outages
        if rand < gpsOutageProbability
            gpsMeasurement = [NaN; NaN];
            if t - lastFollowerValidGPSUpdate > gpsOutageTimeout
                gpsOutageActive = true;
                disp('Persistent GPS Outage!');
            end
        else
            gpsMeasurement = follower1Ekf.state(1:2) + 0.1 * randn(2, 1);
            % Clamp GPS measurements to map size
            gpsMeasurement = min(max(gpsMeasurement, 0), environment.envSize(1));
            lastFollowerValidGPSUpdate = t;
            gpsOutageActive = false;
        end

        [follower1Ekf.state, follower1Ekf.P] = ekf_update_follower(follower1Ekf.state, follower1Ekf.P, gpsMeasurement, follower1Ekf.R_GPS);

        
        % Store Data for
        follower1Trajectory = [follower1Trajectory; follower1Ekf.state(1:2)'];


%% 
        % Update the plot less frequently to reduce flickering
        plot_counter = plot_counter + 1;
        if plot_counter >= plot_update_interval
            % Plotting (Now inside a conditional statement)
            if t > 0  % Plot only after the first iteration

                  % Plot obstacles
                for i = 1:environment.numObstacles
                    obs = obstacles{i};
                    rectangle('Position', [obs.x - obs.radius, obs.y - obs.radius, obs.radius * 2, obs.radius * 2], ...
                        'Curvature', [1, 1], 'FaceColor', [1, 1, 0.7]);
                end

                plot(environment.start(1), environment.start(2), 'go', 'MarkerSize', 10, 'MarkerFaceColor', 'g'); % Start
                plot(environment.goal(1), environment.goal(2), 'ro', 'MarkerSize', 10, 'MarkerFaceColor', 'r'); % Goal

                % Plot the a_star path
                plot(astar_path(:,1), astar_path(:,2), 'bo-', 'LineWidth', 1, 'MarkerSize', 3, 'MarkerFaceColor', 'b');

                % Plot leader trajectory
                plot(leaderTrajectory(:, 1), leaderTrajectory(:, 2), 'g-', 'LineWidth', 2);

                % Plot follower 1 trajectory
                plot(follower1Trajectory(:, 1), follower1Trajectory(:, 2), 'r-', 'LineWidth', 2);

                % Draw the leader robot
                x = leaderEkf.state(1);    % x position
                y = leaderEkf.state(2);    % y position
                theta = leaderEkf.state(3); % heading angle (in radians)
                t_circle = linspace(0, 2*pi, 50);
                x_circle = x + robot.robotRadius * cos(t_circle);
                y_circle = y + robot.robotRadius * sin(t_circle);
                plot(x_circle, y_circle, 'g-', 'LineWidth', 2);
 
                % Length of heading line (tune this as you like)
                heading_length = 0.5;  % 1 meter long line
                
                % Compute end point of the heading line
                x_end = x + heading_length * cos(theta);
                y_end = y + heading_length * sin(theta);
                
                % Plot the heading line
                plot([x, x_end], [y, y_end], 'r-', 'LineWidth', 2);

                 % Draw the follower 1 robot
                theta_follower1 = follower1Ekf.state(3);
                x_follower1 = follower1Ekf.state(1);
                y_follower1 = follower1Ekf.state(2);
                x_circle_follower1 = x_follower1 + robot.robotRadius * cos(t_circle);
                y_circle_follower1 = y_follower1 + robot.robotRadius * sin(t_circle);
                plot(x_circle_follower1, y_circle_follower1, 'r-', 'LineWidth', 2);


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
        plot(environment.start(1), environment.start(2), 'go', 'MarkerSize', 10, 'MarkerFaceColor', 'g'); % Start
        plot(environment.goal(1), environment.goal(2), 'ro', 'MarkerSize', 10, 'MarkerFaceColor', 'r'); % Goal

        % Plot the a_star path
        %plot(astar_path(:,1), astar_path(:,2), 'bo-', 'LineWidth', 1, 'MarkerSize', 3, 'MarkerFaceColor', 'b');
        % end
    end

    % After the loop, plot the final trajectory and filtered trajectory

    % Plot obstacles
    for i = 1:environment.numObstacles
        obs = obstacles{i};
        rectangle('Position', [obs.x - obs.radius, obs.y - obs.radius, obs.radius * 2, obs.radius * 2], ...
            'Curvature', [1, 1], 'FaceColor', [1, 1, 0.7]);
    end

    plot(leaderTrajectory(:, 1), leaderTrajectory(:, 2), 'b-', 'LineWidth', 2);
    plot(follower1Trajectory(:, 1), follower1Trajectory(:, 2), 'b-', 'LineWidth', 2);
   

      % Draw the final position of the leader robot
    theta = leaderEkf.state(3);
    x = leaderEkf.state(1);
    y = leaderEkf.state(2);
    t_circle = linspace(0, 2*pi, 50);
    x_circle = x + robot.robotRadius * cos(t_circle);
    y_circle = y + robot.robotRadius * sin(t_circle);
    plot(x_circle, y_circle, 'g-', 'LineWidth', 2);

    % Draw the final position of the follower 1 robot
    theta_follower1 = follower1Ekf.state(3);
    x_follower1 = follower1Ekf.state(1);
    y_follower1 = follower1Ekf.state(2);
    x_circle_follower1 = x_follower1 + robot.robotRadius * cos(t_circle);
    y_circle_follower1 = y_follower1 + robot.robotRadius * sin(t_circle);
    plot(x_circle_follower1, y_circle_follower1, 'r-', 'LineWidth', 2);

    % Plot settings
    xlim([0 environment.envSize(1)]);
    ylim([0 environment.envSize(2)]);
    xlabel('X Position');
    ylabel('Y Position');
    title('Final Leader and Follower Robot Trajectories');
    hold off;

    figure;
    hold on;
    plot(leaderTrueTrajectory(:, 1), leaderTrueTrajectory(:, 2), 'r-', 'LineWidth', 2);
    plot(followerTrueTrajectory(:, 1), followerTrueTrajectory(:, 2), 'b-', 'LineWidth', 2);

    % Plot settings
    xlim([0 environment.envSize(1)]);
    ylim([0 environment.envSize(2)]);
    xlabel('X Position');
    ylabel('Y Position');
    title('Final Leader and Follower Robot True Trajectories');
    hold off;

    figure;
    for i = 1:environment.numGrids(1)
        for j = 1:environment.numGrids(2)
            if gridValues(i, j) == 1
                rectangle('Position', [(i-1)*environment.gridSize, (j-1)*environment.gridSize, environment.gridSize, environment.gridSize], ...
                    'EdgeColor', 'k');
                text(i - 0.5, j - 0.5, num2str(gridValues(i, j)), ...
                    'HorizontalAlignment', 'center', 'VerticalAlignment', 'middle');
            end
        end
    end
    axis equal; % Ensures the grid squares are proportional
    xlim([0, environment.numGrids(1) * environment.gridSize]);
    ylim([0, environment.numGrids(2) * environment.gridSize]);
    axis manual;
    xlabel('X position');
    ylabel('Y position');
    title('Obstacle mapping');


    activeLeader = false;               
end