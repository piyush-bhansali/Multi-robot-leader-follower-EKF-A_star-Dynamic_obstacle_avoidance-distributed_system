function [obs_detected, gridValues] = findObs(robot_pose,  obstacles, gridValues, robot, env)

sensor_angle = pi/8;
detected_distances = 10000;
sensor_noise_level = 0.01;
heading_direction = robot_pose(3);
obs_detected = false;

for angle_offset = -sensor_angle:0.1:sensor_angle
    % Compute LiDAR scan angle relative to robot's heading
    scan_angle = heading_direction + angle_offset;

    % Simulate the LiDAR pulse traveling out from the robot
    for dist = 0:0.1:robot.lookAheadDistance
        % Calculate LiDAR detection point
        lidar_x = robot_pose(1) + dist * cos(scan_angle);
        lidar_y = robot_pose(2) + dist * sin(scan_angle);
        % plot([robot_pose(1), lidar_x], [robot_pose(2), lidar_y], 'r--', 'LineWidth', 1); % Scan line in red

        for i = 1:size(obstacles, 1)
            obs = obstacles{i};
            obstacle_dist = sqrt((lidar_x - obs.x)^2 + (lidar_y - obs.y)^2);
            obstacle_dist = obstacle_dist + sensor_noise_level * randn;
            if obstacle_dist < detected_distances
                obsIdx = i;
                detected_distances = obstacle_dist;
            end
        end
    end

    if  detected_distances < robot.robotRadius + obs.radius;
        obs_detected = true;
        gridValues = update_grid(gridValues, obstacles, obsIdx, env);
        return;
    end

end
end
