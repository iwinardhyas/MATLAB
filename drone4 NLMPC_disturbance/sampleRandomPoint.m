function randPoint = sampleRandomPoint(map)
    % Get the dimensions of the map
    [xMax, yMax, zMax] = size(map);

    % Generate a random point within the map bounds
    randPoint = [rand() * xMax, rand() * yMax, rand() * zMax];
end
