%% UKF for Actuator Degradation Detection

% --- 1. Konfigurasi Sistem dan UKF ---
dt = 0.01;        % Waktu sampling (detik)
T_sim = 20;       % Total waktu simulasi
N_steps = T_sim / dt; % Jumlah langkah
g = 9.81;         % Gravitasi (m/s^2)
m = 1.0;          % Massa drone (kg)

% State vector dimensi: [Z, Vz, eta]
% eta: Faktor efisiensi aktuator (1 = sehat, <1 = degradasi)
nx = 3; 
ny = 1; % Pengukuran: [Z] (Posisi vertikal)

% Inisialisasi UKF: [Z_awal, Vz_awal, eta_awal]
x_ukf = [0; 0; 1.0]; % Diasumsikan mulai dari origin, aktuator sehat

% Kovariansi Estimasi Awal (P0)
% Berikan kovariansi tinggi pada estimasi awal eta (faktor efisiensi)
P_ukf = diag([0.01, 0.01, 0.01]); 

% Matriks Kovariansi Noise Proses (Q)
% Q harus non-zero untuk eta agar UKF dapat melacak degradasi (random walk)
Q_ukf = diag([0.001, 0.01, 1e-5]); % [Noise_Z, Noise_Vz, Noise_eta]

% Matriks Kovariansi Noise Pengukuran (R)
R_ukf = diag([0.5]); % Noise pengukuran Z

% Parameter UKF (Unscented Transform)
alpha = 1e-3; kappa = 0; beta = 2;
lambda = alpha^2 * (nx + kappa) - nx;
Wm = [lambda / (nx + lambda); 0.5 / (nx + lambda) * ones(2*nx, 1)];
Wc = Wm;
Wc(1) = Wc(1) + (1 - alpha^2 + beta);

% --- 2. Model Dinamika dan Pengukuran ---

% Model Dinamika UKF (f_ukf): 
% Menjelaskan bagaimana state berevolusi.
% F(x, u) = [Z_dot, Vz_dot, eta_dot]
% Vz_dot = (eta * T_commanded - m*g) / m
% eta_dot = 0 (dalam model prediksi UKF, eta diasumsikan konstan)

% Input u diasumsikan sebagai Thrust yang diperintahkan (Commanded Thrust)
f_ukf = @(x, u) [
    x(1) + x(2)*dt;                          % Z_next
    x(2) + ((x(3) * u - m*g) / m) * dt;      % Vz_next
    x(3)                                     % eta_next = eta tetap
];


% Model Pengukuran UKF (h_ukf): Mengukur posisi Z saja
h_ukf = @(x) x(1);

% --- 3. Inisialisasi Simulasi "Dunia Nyata" ---
x_actual = [0; 0]; % [Z_actual, Vz_actual]
eta_actual_history = zeros(1, N_steps);
history_x_actual = zeros(2, N_steps);
history_x_ukf = zeros(nx, N_steps);
history_y_meas = zeros(ny, N_steps);

% --- 4. Simulasi Loop Utama ---
disp('Simulating and Detecting Actuator Degradation with UKF...');

% Input Thrust yang diperintahkan (agar drone melayang stabil)
% Diasumsikan Thrust yang diperintahkan = m*g (T = 9.81 N)
commanded_thrust = m * g; 

% Waktu terjadinya degradasi (misalnya, pada detik ke-10)
degradation_time = 10;
degradation_factor = 0.5; % Efisiensi motor turun 50%

for k = 1:N_steps
    t = (k-1) * dt;

    % ----------------------------------------------------
    % (A) Simulasi Degradasi Aktuator "Dunia Nyata"
    % ----------------------------------------------------
    if t >= degradation_time
        % Setelah waktu degradasi, efisiensi motor "dunia nyata" turun
        eta_actual = degradation_factor; 
    else
        % Motor sehat pada awal simulasi
        eta_actual = 1.0;
    end
    eta_actual_history(k) = eta_actual;

    % Thrust aktual yang dihasilkan motor
    actual_thrust = eta_actual * commanded_thrust;
    
    % ----------------------------------------------------
    % (B) Simulasi Drone Aktual (Gerakan sebenarnya)
    % ----------------------------------------------------
    % Dinamika vertikal aktual: Z_dot = Vz, Vz_dot = (T_actual - m*g) / m
    
    accel_actual = (actual_thrust - m*g) / m;
    
    % Integrasi Euler untuk mendapatkan state actual berikutnya
    x_actual_next = zeros(2,1);
    x_actual_next(1) = x_actual(1) + x_actual(2) * dt; % Z_next = Z + Vz*dt
    x_actual_next(2) = x_actual(2) + accel_actual * dt; % Vz_next = Vz + Az*dt
    
    x_actual = x_actual_next;
    history_x_actual(:, k) = x_actual;

    % ----------------------------------------------------
    % (C) Pengukuran (Tambahkan noise ke posisi Z aktual)
    % ----------------------------------------------------
    % Y_measured = Z_actual + noise
    y_measured = x_actual(1) + sqrt(R_ukf) * randn(ny, 1);
    history_y_meas(:, k) = y_measured;
    
%     disp(['Actual Thrust: ', num2str(actual_thrust), ' | Actual Eta: ', num2str(eta_actual), ' | Acceleration: ', num2str(accel_actual)]); 

    % ----------------------------------------------------
    % (D) UKF Prediksi dan Update (Estimasi Degradasi)
    % ----------------------------------------------------
    
    % --- Prediksi ---
    sqrtP = chol(P_ukf, 'lower');
    X_sigma = repmat(x_ukf, 1, 2*nx+1);
    X_sigma(:, 2:nx+1) = X_sigma(:, 2:nx+1) + sqrt(nx + lambda) * sqrtP;
    X_sigma(:, nx+2:end) = X_sigma(:, nx+2:end) - sqrt(nx + lambda) * sqrtP;
    
    % Prediksi Sigma Points menggunakan f_ukf
    X_sigma_pred = zeros(nx, 2*nx+1);
    for i = 1:2*nx+1
        X_sigma_pred(:, i) = f_ukf(X_sigma(:, i), commanded_thrust);
    end
    
%     x_pred = X_sigma_pred * Wm;
%     P_pred = Q_ukf; 
%     for i = 1:2*nx+1
%         P_pred = P_pred + Wc(i) * (X_sigma_pred(:, i) - x_pred) * (X_sigma_pred(:, i) - x_pred)';
%     end
    x_pred = zeros(nx,1);
    for i = 1:2*nx+1
        x_pred = x_pred + Wm(i) * X_sigma_pred(:, i);
    end

    P_pred = Q_ukf;
    for i = 1:2*nx+1
        diff = X_sigma_pred(:, i) - x_pred;
        P_pred = P_pred + Wc(i) * (diff * diff');
    end
    
    % --- Update ---
    Y_sigma_pred = zeros(ny, 2*nx+1);
    for i = 1:2*nx+1
        Y_sigma_pred(:, i) = h_ukf(X_sigma_pred(:, i));
    end
    y_pred = Y_sigma_pred * Wm;
    
    Pyy = R_ukf; Pxy = zeros(nx, ny);
    for i = 1:2*nx+1
        Pyy = Pyy + Wc(i) * (Y_sigma_pred(:, i) - y_pred) * (Y_sigma_pred(:, i) - y_pred)';
        Pxy = Pxy + Wc(i) * (X_sigma_pred(:, i) - x_pred) * (Y_sigma_pred(:, i) - y_pred)';
    end
    
    K = Pxy / Pyy;
    inovasi = y_measured - y_pred;
    x_ukf = x_pred + K * inovasi;
    P_ukf = P_pred - K * Pyy * K';

    % Simpan hasil estimasi UKF
    history_x_ukf(:, k) = x_ukf;
    
    if mod(k, 100) == 0  % setiap 1 detik (karena dt=0.01)
        disp(['Time ', num2str(t), 's - Estimasi Eta: ', num2str(x_ukf(3))]);
    end
    
    if any(isnan(x_ukf))
        error('x_ukf contains NaN at time %.2f', t);
    end
    
    if mod(k, 100) == 0
        disp(['Sigma eta pred (t=', num2str(t), 's): ', num2str(X_sigma_pred(3,:))]);
    end



end

% --- 5. Visualisasi Hasil Estimasi Degradasi ---
figure;
subplot(2,1,1);
plot(dt*(1:N_steps), history_x_ukf(3,:), 'b', 'LineWidth', 1.5);
hold on;
plot(dt*(1:N_steps), eta_actual_history, 'r--', 'LineWidth', 1.5);
title('Estimasi Efisiensi Aktuator (eta)');
xlabel('Waktu (s)');
ylabel('Efisiensi');
legend('UKF Estimasi', 'Aktual Degradasi');
grid on;

subplot(2,1,2);
plot(dt*(1:N_steps), history_x_ukf(1,:), 'b', 'LineWidth', 1.5);
hold on;
plot(dt*(1:N_steps), history_x_actual(1,:), 'r--', 'LineWidth', 1.5);
title('Posisi Z: Aktual vs Estimasi');
xlabel('Waktu (s)');
ylabel('Posisi Z (m)');
legend('UKF Estimasi Z', 'Aktual Z');
grid on;