function isFree = isCollisionFree(map, point1, point2)
    % Ensure point1 and point2 are row vectors
    point1 = point1(:)'; % Convert to row vector
    point2 = point2(:)'; % Convert to row vector

    % Define the number of steps for checking
    numSteps = 10;

    % Interpolate points along the path
    steps = linspace(0, 1, numSteps);
    for i = 1:numSteps
        intermediatePoint = point1 + steps(i) * (point2 - point1);

        % Round to integer for indexing the map
        x = round(intermediatePoint(1));
        y = round(intermediatePoint(2));
        z = round(intermediatePoint(3));

        % Check if the point is within bounds and not an obstacle
        if x < 1 || y < 1 || z < 1 || ...
           x > size(map, 1) || y > size(map, 2) || z > size(map, 3) || ...
           map(x, y, z) == 1
            isFree = false;
            return;
        end
    end

    % If all points along the path are free
    isFree = true;
end
