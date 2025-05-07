% Interpolation function for smoothing paths
function interpolated_path = interpolate_path(path, pathProperties)
interpolated_path = [];
for i = 1:size(path, 1) - 1
    start = path(i, :);
    end_point = path(i + 1, :);
    for t = linspace(0, 1, pathProperties.numPoints - 1)
        interpolated_path = [interpolated_path; (1 - t) * start + t * end_point];
    end
end
interpolated_path = [interpolated_path; path(end, :)];
end