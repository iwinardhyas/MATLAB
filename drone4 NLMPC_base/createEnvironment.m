function map = createEnvironment()
    % Define a 3D grid map with obstacles
    mapSize = [20, 20, 20]; % Size of the map (x, y, z dimensions)
    map = zeros(mapSize);   % Initialize map as free space (zeros)

    % Add obstacles in the map
    map(8:12, 8:12, 1:10) = 1; % Cubic obstacle
    map(5:7, 15:18, 3:8) = 1;  % Another obstacle
    map(14:18, 4:7, 5:12) = 1; % Another obstacle

    % Note: Adjust obstacles based on your simulation requirements
end
