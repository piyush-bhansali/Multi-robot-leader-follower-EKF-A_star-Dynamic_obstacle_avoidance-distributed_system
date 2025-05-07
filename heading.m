% Find the closest point on the path to the robot (using squared distance for efficiency)
function [desired_heading, lookahead_point_idx, distances, deviation] = heading(robot_pose, path, pathProperties, robot)
deviation = 0;
distances = sqrt(sum((path(:, 1:2) - robot_pose(1:2)).^2, 2));
[~, closest_point_idx] = min(distances);

% Safety check if robot deviates too far from path
if sqrt(min(distances)) > 2 * pathProperties.nextNodeDist
    warning('Robot deviating from path');
    deviation = 1;
    desired_heading = NaN;
    lookahead_point_idx = NaN;
    return;
end

% Find the lookahead point on the path
lookahead_point_idx = closest_point_idx;
while lookahead_point_idx < size(path, 1) && distances(lookahead_point_idx) < pathProperties.nextNodeDist^2
    lookahead_point_idx = lookahead_point_idx + 1;
end

% Make sure lookahead_point_idx doesn't exceed the path size
if lookahead_point_idx > size(path, 1)
    lookahead_point_idx = size(path, 1);
end

lookahead_point = path(lookahead_point_idx, :);

% Calculate desired heading
desired_heading = atan2(lookahead_point(2) - robot_pose(2), lookahead_point(1) - robot_pose(1));
end
