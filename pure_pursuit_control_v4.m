function [v, omega] = pure_pursuit_control_v4(robot_pose, path, obstacles, gridValues, robot, env, pathProperties, followerLost)
% Global variables for path planning
global astar_path;
global newPath;

% Handle follower lost case
if followerLost
    v = 0;
    omega = 0;
    return;
end

% Error handling for empty path
if isempty(path)
    warning('Empty path in pure pursuit control');
    v = 0;
    omega = 0;
    newPath = true;
    return;
end

% Ensure robot_pose is in the correct format
robot_pose = robot_pose(:);  % Make column vector for consistency

% Get desired heading and path information
try
    [desired_heading, lookahead_point_idx, distances, deviation] = heading(robot_pose, path, pathProperties, robot);
catch err
    disp(['Error in heading calculation: ', err.message]);
    v = 0;
    omega = 0;
    return;
end

% Check for obstacles in the path
try
    [obs_detected, gridValues] = findObs(robot_pose, desired_heading, obstacles, gridValues, robot, env);
catch err
    disp(['Error in obstacle detection: ', err.message]);
    obs_detected = false;
end

% Handle obstacle detection or path deviation
if obs_detected || deviation == 1
    % Replan path using A*
    disp('Replanning path due to obstacle or deviation');
    
    % Make sure robot_pose is in the correct format for astar
    robot_pose_for_astar = robot_pose(1:2)';
    
    % Plan new path
    try
        astar_path = astar(robot_pose_for_astar, env, gridValues, pathProperties);
        newPath = true;
        
        % Get new heading after replanning
        [desired_heading, lookahead_point_idx, ~, new_deviation] = heading(robot_pose, astar_path, pathProperties, robot);
        
        if new_deviation == 1 || isnan(desired_heading)
            % If still having issues, slow down and rotate in place
            v = 0;
            omega = 0.1; % Small rotation to explore
            return;
        end
        
        % Calculate steering angle with proper normalization
        steering_angle = wrapToPi(desired_heading - robot_pose(3));
        
        % Limit steering angle to reasonable values
        steering_angle = max(min(steering_angle, pi/4), -pi/4);  % Limit to ±45°
        
        % During replanning, slow down but don't completely stop
        v = robot.maxSpeed * 0.3;  % Reduce speed to 30%
        omega = steering_angle * robot.maxSpeed / robot.axleLength;
        
        % Ensure omega is within bounds
        omega = max(min(omega, robot.maxOmega), -robot.maxOmega);
        
    catch err
        % Handle path planning errors gracefully
        warning('Path planning failed: %s', err.message);
        v = 0;
        omega = 0;
    end
else
    % Normal pure pursuit behavior
    
    % Calculate steering angle with proper normalization
    steering_angle = wrapToPi(desired_heading - robot_pose(3));
    
    % Calculate appropriate linear velocity based on distance and curvature
    curve_factor = abs(steering_angle) / (pi/2);  % 0 (straight) to 1 (sharp turn)
    speed_factor = 1 - 0.7 * curve_factor;  % Reduce speed in curves
    
    % Set linear and angular velocity
    v = min(robot.maxSpeed * speed_factor, distances(lookahead_point_idx) / 0.2);
    
    % Calculate angular velocity with proper scaling
    omega = steering_angle * v / (robot.axleLength * 0.7);  % Better pure pursuit gain
    
    % Ensure omega is within bounds
    omega = max(min(omega, robot.maxOmega), -robot.maxOmega);
    
    newPath = false;
end

% Additional safety checks
if isnan(v) || isnan(omega)
    warning('NaN values detected in pure pursuit control!');
    v = 0;
    omega = 0;
end

% Ensure minimum velocity for rotation when needed
if abs(steering_angle) > pi/6 && v < 0.1
    v = 0.1;  % Minimum velocity to ensure rotation happens
end

end