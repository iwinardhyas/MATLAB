function [x_store, Q_hist, R_hist, innovation_hist,P_hist] = runRAUKF(...
    x0, P0, Q0, R0, measurements, dt, chi2_thr, lambda0, delta0, a, b,dynamics_fcn,measurement_fcn)

    % Dimensi
    [nz, Nsim] = size(measurements);
    nx = length(x0);

    % Inisialisasi variabel
    x_est = x0;
    P = P0;
    Q = Q0;
    R = R0;

    % Penyimpanan hasil
    x_store = zeros(nx, Nsim);
    Q_hist  = zeros(nx, nx, Nsim);
    R_hist  = zeros(nz, nz, Nsim);
    P_hist  = zeros(nx, nx, Nsim);
    innovation_hist = zeros(nz, Nsim);

    % Loop RAUKF
    for k = 1:Nsim
        %% 1. Generate sigma points
        [Xsig, W] = sigma_points(x_est, P);

        %% 2. Prediksi state
        X_pred = zeros(nx, size(Xsig, 2));
        for i = 1:size(Xsig, 2)
            X_pred(:, i) = dynamics_fcn(Xsig(:, i), dt);
        end
        x_pred = X_pred * W';
        P_pred = Q;
        for i = 1:size(Xsig, 2)
            diff = X_pred(:, i) - x_pred;
            P_pred = P_pred + W(i) * (diff * diff');
        end


        %% 3. Prediksi measurement
        Zsig = zeros(nz, size(Xsig, 2));
        for i = 1:size(Xsig, 2)
            Zsig(:, i) = measurement_fcn(X_pred(:, i));
        end
        z_pred = Zsig * W';
        Pzz = R;
        Pxz = zeros(nx, nz);
        for i = 1:size(Xsig, 2)
            dz = Zsig(:, i) - z_pred;
            dx = X_pred(:, i) - x_pred;
            Pzz = Pzz + W(i) * (dz * dz');
            Pxz = Pxz + W(i) * (dx * dz');
        end

        %% 4. Update state
        K = Pxz / Pzz;
        z_meas = measurements(:, k);
        innov = z_meas - z_pred;
        x_upd = x_pred + K * innov;
        P_upd = P_pred - K * Pzz * K';

        % Simpan inovasi
        innovation_hist(:, k) = innov;

        %% 5. Fault detection (Adaptive Q & R)
        phi = innov' * (Pzz \ innov);
        if phi > chi2_thr
            % Adaptive Q
            lambda = max(lambda0, (phi - a * chi2_thr) / phi);
            Q = (1 - lambda) * Q + lambda * (K * innov * innov' * K');

            % Adaptive R
            delta = max(delta0, (phi - b * chi2_thr) / phi);
            residual = z_meas - measurement_fcn(x_upd);
            Szz_hat = cov(Zsig');
            R = (1 - delta) * R + delta * (residual * residual' + Szz_hat);

            % Koreksi estimasi
            Pxx_corr = P_upd + Q;
            Pxz_corr = Pxz;
            Pzz_corr = Szz_hat + R;
            K_hat = Pxz_corr / Pzz_corr;
            x_upd = x_upd + K_hat * (z_meas - z_pred);
            P_upd = Pxx_corr - K_hat * Pzz_corr * K_hat';
        end

        %% Simpan hasil
        x_est = x_upd;
        P = P_upd;
        x_store(:, k) = x_est;
        Q_hist(:, :, k) = Q;
        R_hist(:, :, k) = R;
        P_hist(:, :, k) = P;
    end
end
