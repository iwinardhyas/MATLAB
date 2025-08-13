function path = extractPath(tree, goal)
    % Extract path from tree to goal
    % tree: List of nodes with fields 'location' and 'parent'
    % goal: Goal point (3D coordinates)
    
    % Cari node goal di tree
    currentNode = findNode(tree, goal); 
    if isempty(currentNode)
        error('Goal node not found in tree.');
    end
    
    % Backtrack dari goal ke root tree
    path = [];
    while ~isempty(currentNode)
        path = [tree(currentNode).location; path]; % Tambahkan point ke path
        currentNode = tree(currentNode).parent;    % Lanjut ke parent
    end
end

function idx = findNode(tree, point)
    % Helper function untuk mencari indeks node berdasarkan lokasi
    tolerance = 1e-3; % Toleransi perbandingan
    idx = [];
    for i = 1:length(tree)
        if norm(tree(i).location - point) < tolerance
            idx = i;
            break;
        end
    end
end
