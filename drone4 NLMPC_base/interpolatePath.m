function smoothedPath = interpolatePath(path, numPoints)
    % Initialize the smoothed path
    smoothedPath = [];

    % Iterate over each segment in the path
    for i = 1:(size(path, 1) - 1)
        % Linearly interpolate between consecutive points
        segment = linspace(0, 1, numPoints)';
        smoothedPath = [smoothedPath; ...
            path(i, :) + segment .* (path(i + 1, :) - path(i, :))];
    end
end
