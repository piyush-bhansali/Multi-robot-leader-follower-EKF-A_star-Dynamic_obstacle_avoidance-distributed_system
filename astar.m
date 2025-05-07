function astar_path = astar(robotPose, env, gridValues, pathProperties)
start = robotPose(1:2);
grid_x = 0:env.gridSize:env.numGrids; % X-coordinates of grid
grid_y = 0:env.gridSize:env.numGrids; % Y-coordinates of grid

[~, start_idx_x] = min(abs(grid_x - start(1)));
[~, start_idx_y] = min(abs(grid_y - start(2)));
[~, goal_idx_x] = min(abs(grid_x - env.goal(1)));
[~, goal_idx_y] = min(abs(grid_y - env.goal(2)));

% Initialize open and closed lists
open_list = [];
closed_list = zeros(size(gridValues));

% Define start node
start_node.x = start_idx_x;
start_node.y = start_idx_y;
start_node.g = 0;
start_node.h = sqrt((start_idx_x - goal_idx_x)^2 + (start_idx_y - goal_idx_y)^2);
start_node.f = start_node.g + start_node.h;
start_node.parent = [];

% Add start node to open list
open_list = [open_list; start_node];

% A* search loop
while ~isempty(open_list)
    [~, idx] = min([open_list.f]);
    current_node = open_list(idx);
    open_list(idx) = [];

    % Check if goal is reached
    if current_node.x == goal_idx_x && current_node.y == goal_idx_y
        disp('Path found!');
        astar_path = reconstruct_path(current_node, grid_x, grid_y, pathProperties);
        return;
    end

    % Add current node to closed list
    closed_list(current_node.x, current_node.y) = 1;

    % Generate neighbors
    neighbors = get_neighbors(current_node);
    for n = 1:length(neighbors)
        neighbor = neighbors(n);

        % Skip if neighbor is out of bounds, in closed list, or in an obstacle
        if neighbor.x < 1 || neighbor.y < 1 || ...
                neighbor.x > size(gridValues, 1) || ...
                neighbor.y > size(gridValues, 2) || ...
                closed_list(neighbor.x, neighbor.y) == 1 || ...
                gridValues(neighbor.x, neighbor.y) == 1
            continue;
        end

        % Calculate costs
        neighbor.g = current_node.g + 1; % Assume uniform cost
        neighbor.h = sqrt((neighbor.x - goal_idx_x)^2 + (neighbor.y - goal_idx_y)^2);
        neighbor.f = neighbor.g + neighbor.h;
        neighbor.parent = current_node;

        % Add or update the neighbor in the open list
        existing_idx = find(arrayfun(@(x) x.x == neighbor.x && x.y == neighbor.y, open_list), 1);
        if isempty(existing_idx)
            open_list = [open_list; neighbor];
        else
            if open_list(existing_idx).g > neighbor.g
                open_list(existing_idx) = neighbor; % Update with lower cost
            end
        end
    end
end
error('No Path found');
end

function astar_path = reconstruct_path(node, grid_x, grid_y, pathProperties)
astar_path = [];
while ~isempty(node)
    astar_path = [grid_x(node.x), grid_y(node.y); astar_path]; % Prepend to path
    node = node.parent;
end
astar_path = interpolate_path(astar_path, pathProperties);
end

function neighbors = get_neighbors(node)
offsets = [-1, 0; 1, 0; 0, -1; 0, 1; -1, -1; -1, 1; 1, -1; 1, 1];
neighbors = struct('x', {}, 'y', {}, 'g', {}, 'h', {}, 'f', {}, 'parent', {});
for i = 1:size(offsets, 1)
    neighbors(i).x = node.x + offsets(i, 1);
    neighbors(i).y = node.y + offsets(i, 2);
end
end