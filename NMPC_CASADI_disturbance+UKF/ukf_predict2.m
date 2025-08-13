function [x_pred, P_pred] = ukf_predict2(x_est, P, u, ukf_dynamics_fcn, Q, dt, g_val, alpha, kappa, beta, l_val, Ixx_val, Iyy_val, Izz_val)
    % ===== 1. Setup Parameter Unscented Transform =====
    nx = numel(x_est);
    lambda = alpha^2 * (nx + kappa) - nx;
    Wm = [lambda / (nx + lambda); 0.5 / (nx + lambda) * ones(2*nx, 1)];
    Wc = Wm;
    Wc(1) = Wc(1) + (1 - alpha^2 + beta);

    % ===== 2. Pastikan P Simetris dan Positif-Definit =====
    P = 0.5 * (P + P');  % Simetris
    min_eig = min(eig(P));
    if min_eig <= 0
        P = P + (abs(min_eig) + 1e-3) * eye(nx);  % Regularisasi adaptif
    end
    try
        sqrtP = chol(P, 'lower');
    catch
        warning('P masih tidak positif definit, menggunakan regularisasi tambahan.');
        P = P + 1e-3 * eye(nx);
        sqrtP = chol(P, 'lower');
    end

    % ===== 3. Generate Sigma Points =====
    X_sigma = repmat(x_est(:), 1, 2*nx+1);
    X_sigma(:, 2:nx+1)      = X_sigma(:, 2:nx+1)      + sqrt(nx + lambda) * sqrtP;
    X_sigma(:, nx+2:end)    = X_sigma(:, nx+2:end)    - sqrt(nx + lambda) * sqrtP;

    % ===== 4. Propagasi Sigma Points via RK4 =====
    X_sigma_pred = zeros(nx, 2*nx+1);
    for i = 1:2*nx+1
        xi = X_sigma(:, i);
        k1 = ukf_dynamics_fcn(xi, u, g_val, l_val, Ixx_val, Iyy_val, Izz_val);
        k2 = ukf_dynamics_fcn(xi + dt/2 * k1, u, g_val, l_val, Ixx_val, Iyy_val, Izz_val);
        k3 = ukf_dynamics_fcn(xi + dt/2 * k2, u, g_val, l_val, Ixx_val, Iyy_val, Izz_val);
        k4 = ukf_dynamics_fcn(xi + dt    * k3, u, g_val, l_val, Ixx_val, Iyy_val, Izz_val);
        X_sigma_pred(:, i) = xi + dt/6 * (k1 + 2*k2 + 2*k3 + k4);
    end

    % ===== 5. Hitung Prediksi Mean & Kovariansi =====
    x_pred = X_sigma_pred * Wm;
    P_pred = Q;  % Tambahkan noise proses
    for i = 1:2*nx+1
        diff = X_sigma_pred(:, i) - x_pred;
        P_pred = P_pred + Wc(i) * (diff * diff');
    end

    % ===== 6. Pastikan P_pred Simetris & Positif-Definit =====
    P_pred = 0.5 * (P_pred + P_pred');
    min_eig_pred = min(eig(P_pred));
    if min_eig_pred <= 0
        P_pred = P_pred + (abs(min_eig_pred) + 1e-3) * eye(nx);
    end
    if any(isnan(P_pred(:))) || any(isinf(P_pred(:)))
        error('NaN/Inf terdeteksi di P sebelum Cholesky');
    end
    if any(isnan(X_sigma_pred(:))) || any(isinf(X_sigma_pred(:)))
        error('NaN/Inf di X_sigma_pred');
    end

end