function newPoint = steer(nearestVertex, randPoint, stepSize)
    % Calculate the direction vector
    direction = randPoint - nearestVertex;

    % Normalize and scale the direction to the step size
    distance = norm(direction);
    if distance > stepSize
        direction = direction / distance * stepSize;
    end

    % Calculate the new point
    newPoint = nearestVertex + direction;
end
