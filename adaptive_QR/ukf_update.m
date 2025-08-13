function [x_upd, P_upd, y_pred, Pyy, Pxy] = ukf_update(x_pred, P_pred, y_meas, h_meas, R, alpha, kappa, beta)
    n = numel(x_pred);
    m = numel(y_meas);
    lambda = alpha^2 * (n + kappa) - n;

    % Sigma points
    [X_sigma, Wm, Wc] = sigma_points(x_pred, P_pred, lambda, n);

    % Prediksi pengukuran
    for i = 1:size(X_sigma,2)
        Y_sigma(:,i) = h_meas(X_sigma(:,i));
    end
    y_pred = Y_sigma * Wm;

    % Kovariansi inovasi
    Pyy = R;
    for i = 1:size(Y_sigma,2)
        dy = Y_sigma(:,i) - y_pred;
        Pyy = Pyy + Wc(i) * (dy * dy');
    end

    % Kovariansi silang
    Pxy = zeros(n,m);
    for i = 1:size(X_sigma,2)
        dx = X_sigma(:,i) - x_pred;
        dy = Y_sigma(:,i) - y_pred;
        Pxy = Pxy + Wc(i) * (dx * dy');
    end

    % Gain Kalman
    K = Pxy / Pyy;

    % Update state dan kovariansi
    x_upd = x_pred + K * (y_meas - y_pred);
    P_upd = P_pred - K * Pyy * K';
end
