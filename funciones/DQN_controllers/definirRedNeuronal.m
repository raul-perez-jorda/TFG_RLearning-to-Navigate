function [layers] = definirRedNeuronal(num_entradas, num_acciones)
    layers = [
        featureInputLayer(num_entradas, 'Normalization', 'none', 'Name', 'input')
        fullyConnectedLayer(256, 'Name', 'fc1')
        reluLayer('Name', 'relu1')
        fullyConnectedLayer(256, 'Name', 'fc2')
        reluLayer('Name', 'relu2')
        fullyConnectedLayer(num_acciones, 'Name', 'fc3')
    ];
end