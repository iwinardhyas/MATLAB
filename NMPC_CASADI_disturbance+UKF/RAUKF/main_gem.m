% Skrip utama untuk menjalankan RAUKF
clear; clc;

% Inisialisasi parameter
nx = 12; % Dimensi state
nz = 6;  % Dimensi measurement (sesuaikan)

% Parameter UKF
alpha = 1e-3; % Scaling parameter
beta = 2;     % Parameter untuk Gaussian prior
kappa = 0;    % Sekunder scaling parameter

% Parameter RAUKF
a = 5; % Parameter tuning untuk Q (lihat paper, Bagian 4.1)
b = 5; % Parameter tuning untuk R
lambda0 = 0.2; % Lower limit untuk faktor bobot Q
delta0 = 0.2;  % Lower limit untuk faktor bobot R

% Threshold untuk deteksi fault (dari paper)
chi2_threshold = 2.37; % Untuk 3 derajat kebebasan dan reliabilitas 50%

% Inisialisasi State dan Covariance
x_est = zeros(nx, 1); % Initial state estimation
P_est = eye(nx);      % Initial state covariance
Q = eye(nx) * 0.1;    % Initial Process noise covariance
R = eye(nz) * 0.01;   % Initial Measurement noise covariance

% Loop simulasi
num_steps = 1000;
x_true = zeros(nx, num_steps);
z_measurements = zeros(nz, num_steps);

% Simulasikan data (ganti dengan data drone Anda)
x_true(:, 1) = [0; 0; 0; 0; 0; 0; 0; 0; 0; 0; 0; 0];
for k = 2:num_steps
    x_true(:, k) = drone_state_transition(x_true(:, k-1));
    z_measurements(:, k) = drone_measurement_model(x_true(:, k)) + sqrt(R) * randn(nz, 1);
    % Misalkan ada anomali pada pengukuran setelah langkah 500
    if k > 500
        z_measurements(:, k) = drone_measurement_model(x_true(:, k)) + sqrt(R*100) * randn(nz, 1);
    end
end

% Jalankan filter
x_history = zeros(nx, num_steps);
x_history(:, 1) = x_est;

for k = 2:num_steps
    z_k = z_measurements(:, k);
    [x_est, P_est, Q, R] = RAUKF_step(x_est, P_est, Q, R, z_k, @drone_state_transition, @drone_measurement_model, alpha, beta, kappa, a, b, lambda0, delta0, chi2_threshold);
    x_history(:, k) = x_est;
end

% Plot hasil
figure;
subplot(3,1,1);
plot(x_true(1,:), 'b', 'LineWidth', 2);
hold on;
plot(x_history(1,:), 'r--', 'LineWidth', 2);
title('Posisi X');
legend('True', 'Estimated');

subplot(3,1,2);
plot(x_true(4,:), 'b', 'LineWidth', 2);
hold on;
plot(x_history(4,:), 'r--', 'LineWidth', 2);
title('Posisi Y');
legend('True', 'Estimated');

subplot(3,1,3);
plot(x_true(7,:), 'b', 'LineWidth', 2);
hold on;
plot(x_history(7,:), 'r--', 'LineWidth', 2);
title('Posisi Z');
legend('True', 'Estimated');

function z_k = drone_measurement_model(x_curr)
    % z = [x, y, z, phi, theta, psi] (contoh: dari GPS dan IMU)
    % Model ini harus sesuai dengan sensor Anda.
    z_k = zeros(6, 1);
    z_k(1) = x_curr(1); % Posisi X
    z_k(2) = x_curr(4); % Posisi Y
    z_k(3) = x_curr(7); % Posisi Z
    z_k(4) = x_curr(10); % Sudut phi
    z_k(5) = x_curr(11); % Sudut theta
    z_k(6) = x_curr(12); % Sudut psi
end

function x_next = drone_state_transition(x_curr)
    % x = [x, x_dot, x_ddot, y, y_dot, y_ddot, z, z_dot, z_ddot, phi, theta, psi]
    % (Posisi, Kecepatan, Percepatan, Sudut Euler)
    % Ini adalah contoh model, Anda harus menggantinya dengan model NMPC Anda.
    
    dt = 0.01; % Ganti dengan waktu sampling Anda
    
    A = eye(12);
    A(1,2) = dt; A(1,3) = 0.5*dt^2;
    A(2,3) = dt;
    A(4,5) = dt; A(4,6) = 0.5*dt^2;
    A(5,6) = dt;
    A(7,8) = dt; A(7,9) = 0.5*dt^2;
    A(8,9) = dt;
    A(10,11) = dt;
    A(11,12) = dt;

    x_next = A * x_curr;
end

function [x_k_k, P_k_k, Q, R] = RAUKF_step(x_prev, P_prev, Q_prev, R_prev, z_k, f, h, alpha, beta, kappa, a, b, lambda0, delta0, chi2_threshold)
    % RAUKF_step: Melakukan satu langkah filter RAUKF.
    % Berdasarkan paper "A Robust Adaptive Unscented Kalman Filter" (sensors-18-00808.pdf)

    nx = size(x_prev, 1); % Dimensi state
    nz = size(z_k, 1);    % Dimensi measurement

    % --- Bagian 1: Generate Sigma Points dan Bobot ---
    % Disesuaikan dari Persamaan (4) dan (5)
    lambda = alpha^2 * (nx + kappa) - nx;
    gamma = sqrt(nx + lambda);

    % Bobot untuk mean
    weights_m = zeros(1, 2*nx + 1);
    weights_m(1) = lambda / (nx + lambda);
    weights_m(2:end) = 1 / (2 * (nx + lambda));

    % Bobot untuk covariance
    weights_c = zeros(1, 2*nx + 1);
    weights_c(1) = lambda / (nx + lambda) + (1 - alpha^2 + beta);
    weights_c(2:end) = 1 / (2 * (nx + lambda));
    
    % Menciptakan matriks sigma points
    sigma_points = zeros(nx, 2*nx + 1);
    sigma_points(:, 1) = x_prev;
    P_sqrt = real(chol(P_prev))';
    for i = 1:nx
        sigma_points(:, i+1) = x_prev + gamma * P_sqrt(:, i);
        sigma_points(:, i+nx+1) = x_prev - gamma * P_sqrt(:, i);
    end

    % --- Bagian 2: Prediksi State (Predict) ---
    % Disesuaikan dari Persamaan (8) dan (9)
    predicted_sigma_points = arrayfun(f, sigma_points, 'UniformOutput', false);
    predicted_sigma_points = cell2mat(predicted_sigma_points);

    % KOREKSI: Menggunakan perkalian matriks untuk weighted sum
    x_bar_k_k_minus_1 = predicted_sigma_points * weights_m';

    P_bar_k_k_minus_1 = Q_prev;
    for i = 1:(2*nx + 1)
        P_bar_k_k_minus_1 = P_bar_k_k_minus_1 + weights_c(i) * (predicted_sigma_points(:, i) - x_bar_k_k_minus_1) * (predicted_sigma_points(:, i) - x_bar_k_k_minus_1)';
    end

    % --- Bagian 3: Prediksi Pengukuran (Update) ---
    % Disesuaikan dari Persamaan (10), (11), (12), (13)
    P_bar_sqrt = real(chol(P_bar_k_k_minus_1))';
    sigma_points_predicted = zeros(nx, 2*nx + 1);
    sigma_points_predicted(:, 1) = x_bar_k_k_minus_1;
    for i = 1:nx
        sigma_points_predicted(:, i+1) = x_bar_k_k_minus_1 + gamma * P_bar_sqrt(:, i);
        sigma_points_predicted(:, i+nx+1) = x_bar_k_k_minus_1 - gamma * P_bar_sqrt(:, i);
    end
    
    predicted_measurements = arrayfun(h, sigma_points_predicted, 'UniformOutput', false);
    predicted_measurements = cell2mat(predicted_measurements);

    % KOREKSI: Menggunakan perkalian matriks untuk weighted sum
    z_bar_k_k_minus_1 = predicted_measurements * weights_m';

    P_zz = R_prev;
    P_xz = zeros(nx, nz);
    for i = 1:(2*nx + 1)
        P_zz = P_zz + weights_c(i) * (predicted_measurements(:, i) - z_bar_k_k_minus_1) * (predicted_measurements(:, i) - z_bar_k_k_minus_1)';
        P_xz = P_xz + weights_c(i) * (predicted_sigma_points(:, i) - x_bar_k_k_minus_1) * (predicted_measurements(:, i) - z_bar_k_k_minus_1)';
    end
    
    % --- Bagian 4: Pembaruan State dan Covariance ---
    % Disesuaikan dari Persamaan (14), (15), (16)
    K = P_xz / P_zz;
    x_k_k = x_bar_k_k_minus_1 + K * (z_k - z_bar_k_k_minus_1);
    P_k_k = P_bar_k_k_minus_1 - K * P_zz * K';

    % --- Bagian 5: Mekanisme Adaptif RAUKF ---
    % Berdasarkan Algoritma 1, Persamaan (17), (21), (22), (29), (30)
    mu_k = z_k - z_bar_k_k_minus_1; % Innovation
    
    % Fault-detection mechanism
    varphi_k = mu_k' * inv(P_zz) * mu_k;
    
    if varphi_k > chi2_threshold
        % Update Q
        lambda_factor = max(lambda0, (varphi_k - a * chi2_threshold) / varphi_k);
        Q = (1 - lambda_factor) * Q_prev + lambda_factor * (K * mu_k * mu_k' * K');
        
        % Update R
        epsilon_k = z_k - h(x_k_k); % Residual vector
        
        % Generate sigma points and predicted measurements for correction
        P_sqrt_k_k = real(chol(P_k_k))';
        sigma_points_corrected = zeros(nx, 2*nx + 1);
        sigma_points_corrected(:, 1) = x_k_k;
        for i = 1:nx
            sigma_points_corrected(:, i+1) = x_k_k + gamma * P_sqrt_k_k(:, i);
            sigma_points_corrected(:, i+nx+1) = x_k_k - gamma * P_sqrt_k_k(:, i);
        end
        
        predicted_measurements_corrected = arrayfun(h, sigma_points_corrected, 'UniformOutput', false);
        predicted_measurements_corrected = cell2mat(predicted_measurements_corrected);
        
        % KOREKSI: Menggunakan perkalian matriks untuk weighted sum
        z_hat_k_k = predicted_measurements_corrected * weights_m';
        
        S_k_k_zz = zeros(nz, nz);
        for i = 1:(2*nx + 1)
            S_k_k_zz = S_k_k_zz + weights_c(i) * (predicted_measurements_corrected(:, i) - z_hat_k_k) * (predicted_measurements_corrected(:, i) - z_hat_k_k)';
        end
        
        delta_factor = max(delta0, (varphi_k - b * chi2_threshold) / varphi_k);
        R = (1 - delta_factor) * R_prev + delta_factor * (epsilon_k * epsilon_k' + S_k_k_zz);
    else
        % Jika tidak ada fault, gunakan Q dan R sebelumnya
        Q = Q_prev;
        R = R_prev;
    end

    % Lakukan koreksi akhir (opsional, seperti yang dijelaskan dalam paper)
    % Disesuaikan dari Persamaan (31) hingga (35)
    
    % Re-calculate cross-covariance and innovation covariance using updated Q and R
    % Note: P_bar_k_k_minus_1 is re-calculated with updated Q.
    P_bar_k_k_minus_1_updated = Q + P_bar_k_k_minus_1 - Q_prev;
    P_bar_sqrt_updated = real(chol(P_bar_k_k_minus_1_updated))';
    sigma_points_predicted_updated = zeros(nx, 2*nx + 1);
    sigma_points_predicted_updated(:, 1) = x_bar_k_k_minus_1;
    for i = 1:nx
        sigma_points_predicted_updated(:, i+1) = x_bar_k_k_minus_1 + gamma * P_bar_sqrt_updated(:, i);
        sigma_points_predicted_updated(:, i+nx+1) = x_bar_k_k_minus_1 - gamma * P_bar_sqrt_updated(:, i);
    end

    predicted_measurements_updated = arrayfun(h, sigma_points_predicted_updated, 'UniformOutput', false);
    predicted_measurements_updated = cell2mat(predicted_measurements_updated);
    
    % KOREKSI: Menggunakan perkalian matriks untuk weighted sum
    z_bar_k_k_minus_1_updated = predicted_measurements_updated * weights_m';

    P_zz_updated = R + P_zz - R_prev;
    P_xz_updated = zeros(nx, nz);
    for i = 1:(2*nx + 1)
        P_xz_updated = P_xz_updated + weights_c(i) * (predicted_sigma_points(:, i) - x_bar_k_k_minus_1) * (predicted_measurements_updated(:, i) - z_bar_k_k_minus_1_updated)';
    end

    K_updated = P_xz_updated / P_zz_updated;
    x_k_k = x_bar_k_k_minus_1 + K_updated * (z_k - z_bar_k_k_minus_1_updated);
    P_k_k = P_bar_k_k_minus_1_updated - K_updated * P_zz_updated * K_updated';

end