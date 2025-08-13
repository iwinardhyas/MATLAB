    function net = initialize_maml_nn()
    % Buat NN feedforward 2 hidden layer TANPA regressionLayer
    layers = [
        featureInputLayer(17, 'Name', 'input')
        fullyConnectedLayer(64, 'Name', 'fc1')
        reluLayer('Name', 'relu1')
        fullyConnectedLayer(64, 'Name', 'fc2')
        reluLayer('Name', 'relu2')
        fullyConnectedLayer(13, 'Name', 'output') % Tanpa regressionLayer
    ];
    
    % Buat dlnetwork dari layer (tanpa regressionLayer)
    net = dlnetwork(layers);
end
