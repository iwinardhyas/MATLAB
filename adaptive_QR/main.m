% Parameter Adaptive UKF
lambda_Q = 0.01;  % peluruhan untuk Q
lambda_R = 0.05;  % peluruhan untuk R
Q_est = max(Q_est, 1e-6 * eye(size(Q_est)));
R_est = max(R_est, 1e-6 * eye(size(R_est)));


% Loop utama
for k = 1:N
    % Buat pengukuran dengan noise menggunakan R_est terbaru
    measurement_y = ukf_measurement_fcn(current_state) + randn(ny,1).*sqrt(diag(R_est));

    % Adaptive UKF step
    [ukf_x_est, ukf_P, Q_est, R_est] = adaptive_ukf_step( ...
        ukf_x_est, ukf_P, u_applied_prev, measurement_y, ...
        ukf_dynamics_fcn_handle, ukf_measurement_fcn, ...
        Q_est, R_est, dt_ukf, alpha_ukf, kappa_ukf, beta_ukf, ...
        lambda_Q, lambda_R);

    % Simpan hasil estimasi
    x_est_history(:,k) = ukf_x_est;
end

