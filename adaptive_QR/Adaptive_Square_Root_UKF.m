% --- Inisialisasi ---
nx = 12;                           % State: posisi, orientasi, kecepatan, angular rate
x_true = zeros(nx,1);
x_true(3) = 1;                     % mulai di ketinggian 1 m
x_est = zeros(nx,1);               % estimasi awal
S_est = chol(eye(nx), 'lower');    % square-root covariance awal
Q_est = 1e-3 * eye(nx);            % noise proses awal
R_est = 1e-2 * eye(9);             % noise pengukuran awal
alpha = 0.5; beta = 2; kappa = 0;  % parameter UKF
lambda_Q = 0.01; lambda_R = 0.01;  % learning rate adaptif

% Pastikan Q_est dan R_est tetap positif-definit
Q_est = (Q_est + Q_est') / 2;
[Vq, Dq] = eig(Q_est);
Dq(Dq < 1e-8) = 1e-8;
Q_est = Vq * Dq * Vq';

R_est = (R_est + R_est') / 2;
[Vr, Dr] = eig(R_est);
Dr(Dr < 1e-8) = 1e-8;
R_est = Vr * Dr * Vr';




% --- Input hover ---
N = 500; dt = 0.01; m = 1; g = 9.81;
nu = 6;
u_input = zeros(nu, N);
u_input(3,:) = m * g;              % Hover thrust

% --- Simulasi ---
for k = 1:N
    x_true = full(x_true);
    u = full(u_input(:,k));
    x_true = quad_dynamics_for_ukf(x_true, u_input(:,k), dt);



    % Generate noisy measurement
    y_meas = quad_measurement_model(x_true) + randn(9,1).*sqrt(diag(R_est));

    % Adaptive SR-UKF update
    [x_est, S_est, Q_est, R_est] = adaptive_sr_ukf_step( ...
        x_est, S_est, u_input(:,k), y_meas, ...
        @quad_dynamics_for_ukf, @quad_measurement_model, ...
        Q_est, R_est, dt, alpha, beta, kappa, ...
        lambda_Q, lambda_R);

    % Simpan hasil
    x_true_history(:,k) = x_true;
    x_est_history(:,k) = x_est;
end
