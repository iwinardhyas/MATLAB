function [x_est, P] = ukf_update2(x_pred, P_pred, measurement, measurement_fcn, R, alpha, kappa, beta)
    % ===============================
    % UKF UPDATE STEP (VERSI ROBUST)
    % ===============================
    % x_pred        : Prediksi state (nx x 1)
    % P_pred        : Prediksi kovariansi (nx x nx)
    % measurement   : Data pengukuran (ny x 1)
    % measurement_fcn : Fungsi h(x)
    % R             : Kovariansi noise pengukuran (ny x ny)
    % alpha, kappa, beta : Parameter UKF
    %
    % Output:
    % x_est : State hasil update
    % P     : Kovariansi hasil update
    % ===============================

    nx = length(x_pred);
    ny = length(measurement);

    % ===== 1. Regularisasi P_pred agar tetap positif definit =====
    P_pred = 0.5 * (P_pred + P_pred');
    min_eig = min(eig(P_pred));
    if min_eig <= 1e-6
        P_pred = P_pred + (abs(min_eig) + 1e-3) * eye(nx);
    end

    % ===== 2. Hitung parameter Unscented Transform =====
    lambda = alpha^2 * (nx + kappa) - nx;
    try
        chol_P_pred = chol((nx + lambda) * P_pred, 'lower');
    catch
        warning('P_pred tidak positif definit di UKF UPDATE, tambahkan regularisasi.');
        chol_P_pred = chol((nx + lambda) * (P_pred + 1e-2 * eye(nx)), 'lower');
    end

    % ===== 3. Generate Sigma Points =====
    sigma_points = zeros(nx, 2*nx + 1);
    sigma_points(:, 1) = x_pred;
    for i = 1:nx
        sigma_points(:, i+1)     = x_pred + chol_P_pred(:, i);
        sigma_points(:, i+1+nx)  = x_pred - chol_P_pred(:, i);
    end

    % ===== 4. Weights =====
    Wm = [lambda / (nx + lambda); 0.5 / (nx + lambda) * ones(2*nx, 1)];
    Wc = Wm;
    Wc(1) = Wc(1) + (1 - alpha^2 + beta);

    % ===== 5. Propagasi Sigma Points ke Measurement Space =====
    sigma_measurements = zeros(ny, 2*nx + 1);
    for i = 1:(2*nx + 1)
        sigma_measurements(:, i) = measurement_fcn(sigma_points(:, i));
    end

    % ===== 6. Hitung Prediksi Measurement =====
    y_hat = sigma_measurements * Wm;

    % ===== 7. Hitung Kovariansi Measurement (P_yy) =====
    P_yy = zeros(ny, ny);
    for i = 1:(2*nx + 1)
        diff_y = sigma_measurements(:, i) - y_hat;
        P_yy = P_yy + Wc(i) * (diff_y * diff_y');
    end
    P_yy = P_yy + R;

    % Regularisasi tambahan untuk P_yy
    P_yy = 0.5 * (P_yy + P_yy');
    min_eig_Pyy = min(eig(P_yy));
    if min_eig_Pyy <= 1e-6
        P_yy = P_yy + (abs(min_eig_Pyy) + 1e-3) * eye(ny);
    end

    % ===== 8. Hitung Cross-Covariance (P_xy) =====
    P_xy = zeros(nx, ny);
    for i = 1:(2*nx + 1)
        diff_x = sigma_points(:, i) - x_pred;
        diff_y = sigma_measurements(:, i) - y_hat;
        P_xy = P_xy + Wc(i) * (diff_x * diff_y');
    end

    % ===== 9. Kalman Gain =====
    K = P_xy / P_yy;

    % ===== 10. Update State =====
    x_est = x_pred + K * (measurement - y_hat);

    % ===== 11. Update Kovariansi =====
    P = P_pred - K * P_yy * K';
    P = 0.5 * (P + P'); % Simetris
    min_eig_P = min(eig(P));
    if min_eig_P <= 1e-6
        P = P + (abs(min_eig_P) + 1e-3) * eye(nx);
    end
end
