function [net_updated, current_loss_inner] = update_maml_online(net, x_input, y_true, lr_inner, lr_outer)
    % x_input: [1 x 17] input vector
    % y_true:  [1 x 13] true residual
    % lr_inner: learning rate for fast adaptation
    % lr_outer: learning rate for meta update

    % Convert to dlarray
    x_dl = dlarray(x_input', 'CB');  % Transpose ke [17x1], 'CB': Channel x Batch
    y_dl = dlarray(y_true', 'CB');   % Transpose ke [13x1]

    % ===== INNER LOOP =====
    % Fast adaptation (gradient step pada copy dari net)
    [loss_inner, gradients_inner] = dlfeval(@modelGradients, net, x_dl, y_dl);

    % Clone net for inner update
    net_inner = net;
    for i = 1:size(gradients_inner, 1)
%         layer = gradients_inner.Layer(i);
%         param = gradients_inner.Parameter(i);
%         grad_val = gradients_inner.Value{i};
%         old_val = net_inner.Learnables.Value{i};
%         new_val = old_val - lr_inner * grad_val;
        net_inner.Learnables.Value{i} = net_inner.Learnables.Value{i} - lr_inner * gradients_inner.Value{i};
    end

    % ===== OUTER LOOP =====
    % Compute outer loss using net_inner (the fast adapted net)
    [loss_outer, gradients_outer] = dlfeval(@modelGradients, net_inner, x_dl, y_dl);

    % Apply outer meta-update to the original network
    for i = 1:size(gradients_outer, 1)
%         layer = gradients_outer.Layer(i);
%         param = gradients_outer.Parameter(i);
%         grad_val = gradients_outer.Value{i};
%         old_val = net.Learnables.Value{i};
%         new_val = old_val - lr_outer * grad_val;
        net.Learnables.Value{i} = net.Learnables.Value{i} - lr_outer * gradients_outer.Value{i};
    end

    % Output updated net and current loss
    net_updated = net;
    current_loss_inner = extractdata(loss_inner); % Convert scalar dlarray to double
end

function [loss, gradients] = modelGradients(net, x, y)
%     y_pred = forward(net, x);
    y_pred = net.forward(x);
    loss = mean((y_pred - y).^2, 'all'); % Ensure scalar
    gradients = dlgradient(loss, net.Learnables);
end
