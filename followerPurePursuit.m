function [v, omega] = followerPurePursuit(followerEkf, desiredFollowerPose, error, PID, robot)
    % Extract robot pose from EKF state
    robot_pose = followerEkf.state;  % Since follower1Ekf contains the state vector
    
    % Calculate distance to desired pose
    distance = norm(robot_pose(1:2) - desiredFollowerPose(1:2));
    
    % or even ramp only once you’re farther than your safeDistance:
    if distance > robot.safeDistance
        v = min(robot.maxSpeed, distance/0.5);
        % PID controller for velocity adjustment
        v = v + PID.Kp * error.previousError + PID.Ki * error.integralError + PID.Kd * error.derivativeError;
        % Clamp velocity within bounds
        v = max(0, min(v, robot.maxSpeed));
    else
        v = 0;
        disp("Too close to the leader");
    end
    
    % Calculate desired heading
    desired_heading = atan2(desiredFollowerPose(2) - robot_pose(2), ...
        desiredFollowerPose(1) - robot_pose(1));
    
    % Calculate steering angle (normalized between -π and π)
    steering_angle = atan2(sin(desired_heading - robot_pose(3)), ...
        cos(desired_heading - robot_pose(3)));
    
    % Calculate angular velocity
    omega = v * tan(steering_angle) / robot.axleLength;
    
    % Add safety checks
    if isnan(v) || isnan(omega)
        v = 0;
        omega = 0;
    end
    
    % Add minimum distance check
    min_safe_distance = robot.robotRadius * 2;
    if distance < min_safe_distance
        v = 0;  % Stop if too close
        omega = 0;
    end
    
    end
    