function gridValues = update_grid(grid_values, obstacles, obsIdx, env)
global gridValues;
gridValues = grid_values;
obs = obstacles{obsIdx};

object_gridX_start = max(floor((obs.x - obs.radius) / env.gridSize) +1 , 1);
object_gridY_start = max(floor((obs.y - obs.radius) / env.gridSize) +1 , 1);
object_gridX_end = min(ceil((obs.x + obs.radius) / env.gridSize) , env.numGrids(1));
object_gridY_end = min(ceil((obs.y + obs.radius) / env.gridSize) , env.numGrids(2));

% Update grid values to 1 for all grids covered by the object
for i = object_gridX_start:env.gridSize:object_gridX_end
    for j = object_gridY_start:env.gridSize:object_gridY_end
        % Ensure the grid indices are within bounds
        if i >= 1 && i <= env.numGrids(1) && j >= 1 && j <= env.numGrids(2)
            gridValues(i, j) = 1;
        end
    end
end
end