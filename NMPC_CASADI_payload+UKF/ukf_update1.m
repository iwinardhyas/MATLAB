function [x_updated, P_updated] = ukf_update1(x_pred, P_pred, z_measured, measurement_fcn, R, alpha, kappa, beta)
    nx = length(x_pred);   % Jumlah state
    nz = size(R, 1);       % Jumlah pengukuran (HARUSNYA = 1)

    % Parameter UKF
    lambda = alpha^2 * (nx + kappa) - nx;
    gamma = sqrt(nx + lambda);

    % Bobot
    Wm = [lambda / (nx + lambda), repmat(1 / (2 * (nx + lambda)), 1, 2 * nx)];
    Wc = Wm;
    Wc(1) = Wc(1) + (1 - alpha^2 + beta);

    % Sigma points
    sigma_points_pred = zeros(nx, 2 * nx + 1);
    sigma_points_pred(:,1) = x_pred;
    P_pred = P_pred + eye(size(P_pred)) * 1e-9;
    Psqrt = chol(P_pred, 'lower');
    for i = 1:nx
        sigma_points_pred(:, i+1)     = x_pred + gamma * Psqrt(:,i);
        sigma_points_pred(:, i+1+nx) = x_pred - gamma * Psqrt(:,i);
    end

    % Prediksi pengukuran dari semua sigma point
    sigma_measurements = zeros(nz, 2 * nx + 1);
    for i = 1:2 * nx + 1
        sigma_measurements(:, i) = measurement_fcn(sigma_points_pred(:, i));
    end

    % Rata-rata prediksi pengukuran
    z_pred = sigma_measurements * Wm';

    % Inovasi kovariansi
    Pzz = zeros(nz, nz);
    for i = 1:2 * nx + 1
        dz = sigma_measurements(:, i) - z_pred;
        Pzz = Pzz + Wc(i) * (dz * dz');
    end
    Pzz = Pzz + R;

    % Cross covariance
    Pxz = zeros(nx, nz);
    for i = 1:2 * nx + 1
        dx = sigma_points_pred(:, i) - x_pred;
        dz = sigma_measurements(:, i) - z_pred;
        Pxz = Pxz + Wc(i) * (dx * dz');
    end

    % Gain dan update
    K = Pxz / Pzz;
    innovation = z_measured - z_pred;
    x_updated = x_pred + K * innovation;
    P_updated = P_pred - K * Pzz * K';
end
