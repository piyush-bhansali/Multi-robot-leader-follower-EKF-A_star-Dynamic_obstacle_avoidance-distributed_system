function [rangeMeas, bearingMeas] = lidarMeasurement(leaderPose, followerPose)
% LIDARMEASUREMENT  Simulate a single-beam LiDAR on the follower seeing the leader
%
%   Inputs:
%     followerPose – [x_F; y_F; theta_F]  (meters, radians)
%     leaderPose   – [x_L; y_L; theta_L]  (theta_L unused)
%     params.maxRange     – maximum sensor range (m)
%     params.rangeStd     – standard deviation of range noise (m)
%     params.angleStd     – standard deviation of bearing noise (rad)
%
%   Outputs:
%     rangeMeas    – measured distance (m), NaN if out of range
%     bearingMeas  – measured bearing (rad), relative to follower’s heading,
%                    in (–pi,pi], NaN if out of range

    params = struct('maxRange', 2, 'rangeStd', 0.02, 'angleStd', 0.01);

    xF     = followerPose(1);
    yF     = followerPose(2);
    thetaF = followerPose(3);

    xL     = leaderPose(1);
    yL     = leaderPose(2);

    % true relative vector
    dx = xL - xF;
    dy = yL - yF;

    % true range & bearing
    r_true   = norm(leaderPose(1:2) - followerPose(1:2));
    phi_true = wrapToPi(atan2(dy, dx) - thetaF);

    % check max range
    if r_true > params.maxRange
        rangeMeas   = NaN;
        bearingMeas = NaN;
        return;
    end

    % add noise
    rangeMeas   = r_true   + params.rangeStd * randn();
    bearingMeas = phi_true + params.angleStd * randn();

    % wrap bearing back to (–pi,pi]
    bearingMeas = wrapToPi(bearingMeas);
end
