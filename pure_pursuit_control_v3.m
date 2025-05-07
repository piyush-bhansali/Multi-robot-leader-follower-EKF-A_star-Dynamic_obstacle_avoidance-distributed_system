function [v, omega] = pure_pursuit_control_v3(robot_pose, path, obstacles, gridValues, robot, env, pathProperties, followerLost )
% Ensure robot_pose is a row vector
robot_pose = robot_pose(:)';
global astar_path;
global newPath;

if followerLost == true
    v = 0;
    omega = 0;

else
    [desired_heading, lookahead_point_idx, distances, deviation] = heading(robot_pose, path, pathProperties, robot);
    [obs_detected, gridValues] = findObs(robot_pose, obstacles, gridValues, robot, env);
    if obs_detected == true || deviation == 1
        astar_path = astar(robot_pose, env, gridValues, pathProperties);
        [desired_heading, ~, ~, ~] = heading(robot_pose, astar_path, pathProperties, robot);
        steering_angle = wrapToPi(desired_heading - robot_pose(3));
        v = 0;
        omega = robot.maxSpeed * tan(steering_angle) / robot.axleLength;
        newPath = true;
    else
        % Calculate steering angle
        %steering_angle = wrapToPi(desired_heading - robot_pose(3));
        steering_angle = atan2(sin(desired_heading - robot_pose(3)), cos(desired_heading - robot_pose(3)));
        % Calculate linear velocity
        v = min(robot.maxSpeed, distances(lookahead_point_idx) / 0.1);
        % Calculate angular velocity
        %omega = 0.1;
        omega = min(robot.maxOmega, v * tan(steering_angle) / robot.axleLength);
        newPath = false;
    end
end





