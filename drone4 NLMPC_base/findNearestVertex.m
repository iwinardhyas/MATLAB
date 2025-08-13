function nearestVertex = findNearestVertex(tree, randPoint)
    % Calculate the Euclidean distance from randPoint to all points in the tree
    distances = vecnorm(tree - randPoint, 2, 2);

    % Find the point in the tree with the minimum distance
    [~, idx] = min(distances);
    nearestVertex = tree(idx, :);
end
