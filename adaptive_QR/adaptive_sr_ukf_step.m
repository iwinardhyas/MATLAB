function [x_est, S_est, Q_est, R_est] = adaptive_sr_ukf_step( ...
        x_prev, S_prev, u, y_meas, ...
        f_dyn, h_meas, Q_est, R_est, ...
        dt, alpha, beta, kappa, ...
        lambda_Q, lambda_R)

    % --- Setup ---
    nx = length(x_prev);
    ny = length(y_meas);
    lambda = alpha^2 * (nx + kappa) - nx;

    % --- Generate Sigma Points ---
    S_scaled = chol((nx + lambda)) * S_prev;
    X_sigma = [x_prev, x_prev + S_scaled, x_prev - S_scaled];

    % Weights
    Wm = [lambda / (nx + lambda), repmat(1/(2*(nx + lambda)), 1, 2*nx)];
    Wc = Wm;
    Wc(1) = Wc(1) + (1 - alpha^2 + beta);

    % --- Propagate through Dynamics ---
    X_sigma_pred = zeros(nx, 2*nx+1);
    for i = 1:2*nx+1
        k1 = f_dyn(X_sigma(:,i), u, dt);
        k2 = f_dyn(X_sigma(:,i) + 0.5*dt*k1, u, dt);
        k3 = f_dyn(X_sigma(:,i) + 0.5*dt*k2, u, dt);
        k4 = f_dyn(X_sigma(:,i) + dt*k3, u, dt);
        X_sigma_pred(:,i) = X_sigma(:,i) + dt/6 * (k1 + 2*k2 + 2*k3 + k4);
    end

    % --- Predicted Mean & Covariance (Square-Root) ---
    x_pred = X_sigma_pred * Wm';
    X_diff = (X_sigma_pred - x_pred);
    S_pred = cholupdate_qr(S_prev, X_diff, sqrt(Wc), Q_est);


    % --- Measurement Prediction ---
    Y_sigma = zeros(ny, 2*nx+1);
    for i = 1:2*nx+1
        Y_sigma(:,i) = h_meas(X_sigma_pred(:,i));
    end
    y_pred = Y_sigma * Wm';

    % --- Innovation Covariance & Cross-Covariance ---
    Y_diff = (Y_sigma - y_pred);
    S_yy = cholupdate_qr(chol(R_est), Y_diff, sqrt(Wc), R_est);

    P_xy = X_diff * diag(Wc) * Y_diff';

    % --- Kalman Gain ---
    K = (P_xy / (S_yy * S_yy'));

    % --- Update ---
    x_est = x_pred + K * (y_meas - y_pred);
    S_est = cholupdate_multi(S_pred, K * S_yy, -1);


    % --- Adaptive Q & R ---
    innov = y_meas - y_pred;
    Q_est = (1 - lambda_Q) * Q_est + lambda_Q * (K * innov * innov' * K');
    R_est = (1 - lambda_R) * R_est + lambda_R * (innov * innov');

    % Regularisasi
    Q_est = Q_est + 1e-6 * eye(nx);
    R_est = R_est + 1e-6 * eye(ny);
end
