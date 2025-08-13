function [x_pred, P_pred, X_sigma_pred] = ukf_predict(x_prev, P_prev, u_input, f_dyn, Q, dt, alpha, kappa, beta)
    n = numel(x_prev);
    lambda = alpha^2 * (n + kappa) - n;

    % Sigma points
    [X_sigma, Wm, Wc] = sigma_points(x_prev, P_prev, lambda, n);

    % Propagasi melalui dinamika
    for i = 1:size(X_sigma,2)
        X_sigma_pred(:,i) = f_dyn(X_sigma(:,i), u_input, dt);
    end

    % Prediksi mean
    x_pred = X_sigma_pred * Wm;

    % Prediksi kovariansi
    P_pred = Q;
    for i = 1:size(X_sigma_pred,2)
        dx = X_sigma_pred(:,i) - x_pred;
        P_pred = P_pred + Wc(i) * (dx * dx');
    end
end
