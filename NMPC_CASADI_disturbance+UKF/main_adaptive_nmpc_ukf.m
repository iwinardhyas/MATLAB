%% Simulasi UKF untuk Estimasi Gangguan Angin Acak (Wind Gusts)

% --- 1. Konfigurasi Simulasi ---
dt = 0.01;        % Waktu sampling (detik)
T_sim = 20;       % Total waktu simulasi
N_steps = T_sim / dt; % Jumlah langkah
g = 9.81;         % Gravitasi (m/s^2)

% State vector dimensi: [Px, Py, Pz, Vx, Vy, Vz, Wx, Wy, Wz]
nx = 9; 
ny = 3; % Pengukuran: [Px, Py, Pz] (Misalnya, dari GPS)

% --- 2. Inisialisasi UKF ---
% Estimasi awal state: diasumsikan drone diam, tidak ada angin yang diestimasi.
x_ukf = zeros(nx, 1); 

% Kovariansi Estimasi Awal (P0)
P_ukf = diag([0.1*ones(3,1); 0.5*ones(3,1); 5*ones(3,1)]); % Pos, Vel, Angin (Angin tinggi)

% Matriks Kovariansi Noise Proses (Q)
% Q harus tinggi untuk state Angin (Wx, Wy, Wz) untuk memungkinkan Random Walk
Q_ukf = diag([0.01*ones(3,1); 0.1*ones(3,1); 1.5*ones(3,1)]); 

% Matriks Kovariansi Noise Pengukuran (R)
R_ukf = diag([0.5, 0.5, 0.5]); % Noise GPS

% Parameter UKF (Unscented Transform)
alpha = 1e-3;
kappa = 0;
beta = 2;
lambda = alpha^2 * (nx + kappa) - nx;
Wm = [lambda / (nx + lambda); 0.5 / (nx + lambda) * ones(2*nx, 1)];
Wc = Wm;
Wc(1) = Wc(1) + (1 - alpha^2 + beta);

% --- 3. Model Dinamika dan Pengukuran ---

% Model Dinamika UKF (f_ukf): 
% Mengasumsikan angin konstan (random walk) selama prediksi
% Model ini adalah yang digunakan UKF untuk memprediksi state.
% Input u diasumsikan sebagai percepatan [Ax, Ay, Az] (atau Thrust/m)
f_ukf = @(x, u) [
    % Posisi: P = P + (V + W) * dt
    x(1:3) + dt * (x(4:6) + x(7:9)); 
    % Kecepatan: V = V + (A - g) * dt (A = input percepatan)
    x(4) + dt * u(1);
    x(5) + dt * u(2);
    x(6) + dt * (u(3) - g);
    % Angin (Wx, Wy, Wz): Dipertahankan konstan dalam model prediksi UKF (Random Walk)
    x(7); x(8); x(9)
];

% Model Pengukuran UKF (h_ukf):
% Diasumsikan kita hanya mengukur posisi [Px, Py, Pz]
h_ukf = @(x) x(1:3);

% --- 4. Simulasi "Dunia Nyata" Drone (Untuk menghasilkan data aktual) ---
% Catatan: Ini adalah model yang digunakan untuk menghasilkan data simulasi.
% Kita akan menambahkan random wind gusts di sini.

% Variabel state aktual drone
x_actual = zeros(nx, 1);

% Riwayat (untuk menyimpan hasil)
history_x_ukf = zeros(nx, N_steps);
history_x_actual = zeros(nx, N_steps);
history_y_meas = zeros(ny, N_steps);
history_u_input = zeros(3, N_steps); % Menyimpan input kontrol
history_wind_actual = zeros(3, N_steps); % Menyimpan angin aktual

% --- 5. Simulasi Loop Utama ---
disp('Simulating and Estimating Wind Gusts with UKF...');

for k = 1:N_steps
    % ----------------------------------------------------
    % (A) Simulasi Input dan Gangguan "Dunia Nyata"
    % ----------------------------------------------------
    
    % Input Kontrol Drone (Diasumsikan thrust untuk mengimbangi gravitasi di Z, 0 di X,Y)
    u_actual = [0; 0; g]; 
    history_u_input(:, k) = u_actual;

    % Generasi Gangguan Angin Acak (Wind Gusts)
    % Kita akan menggunakan random walk untuk angin aktual
    if k == 1
        actual_wind = zeros(3,1); % Angin awal
    else
        % Tambahkan noise acak ke angin aktual (simulasi)
        % Ini adalah model random walk untuk angin "dunia nyata"
        wind_process_noise = 0.5 * randn(3,1); % Tambahkan noise yang signifikan
        actual_wind = history_wind_actual(:, k-1) + wind_process_noise; 
    end
    history_wind_actual(:, k) = actual_wind;
    
    % ----------------------------------------------------
    % (B) Simulasi Drone Aktual (Gerakan sebenarnya)
    % ----------------------------------------------------
    
    % Model Dinamika Aktual (Posisi, Kecepatan, Angin)
    % Vx_next = Vx + (Ax_input + Wx_actual) * dt, dll.
    
    % Kita gunakan model f_ukf yang dimodifikasi untuk simulasi dunia nyata, 
    % dengan menggunakan input aktual dan angin aktual.
    x_actual_next = [
        x_actual(1:3) + dt * (x_actual(4:6) + actual_wind);
        x_actual(4) + dt * u_actual(1);
        x_actual(5) + dt * u_actual(2);
        x_actual(6) + dt * (u_actual(3) - g);
        actual_wind
    ];
    
    x_actual = x_actual_next;
    history_x_actual(:, k) = x_actual(1:9);
    
    % ----------------------------------------------------
    % (C) Pengukuran (Tambahkan noise ke posisi aktual)
    % ----------------------------------------------------
    y_measured = h_ukf(x_actual) + sqrt(R_ukf) * randn(ny, 1);
    history_y_meas(:, k) = y_measured;

    % ----------------------------------------------------
    % (D) UKF Prediksi (Time Update)
    % ----------------------------------------------------
    
    % Hitung Sigma Points
    sqrtP = chol(P_ukf, 'lower');
    X_sigma = repmat(x_ukf, 1, 2*nx+1);
    X_sigma(:, 2:nx+1) = X_sigma(:, 2:nx+1) + sqrt(nx + lambda) * sqrtP;
    X_sigma(:, nx+2:end) = X_sigma(:, nx+2:end) - sqrt(nx + lambda) * sqrtP;
    
    % Prediksi Sigma Points menggunakan model dinamika UKF (f_ukf)
    X_sigma_pred = zeros(nx, 2*nx+1);
    for i = 1:2*nx+1
        % UKF menggunakan model dinamisnya untuk memprediksi state berikutnya
        X_sigma_pred(:, i) = f_ukf(X_sigma(:, i), u_actual);
    end
    
    % Hitung mean state yang diprediksi
    x_pred = X_sigma_pred * Wm;
    
    % Hitung Kovariansi Prediksi (P_pred)
    P_pred = Q_ukf; 
    for i = 1:2*nx+1
        P_pred = P_pred + Wc(i) * (X_sigma_pred(:, i) - x_pred) * (X_sigma_pred(:, i) - x_pred)';
    end

    % ----------------------------------------------------
    % (E) UKF Update (Measurement Update)
    % ----------------------------------------------------
    
    % Prediksi Pengukuran dari Sigma Points
    Y_sigma_pred = zeros(ny, 2*nx+1);
    for i = 1:2*nx+1
        Y_sigma_pred(:, i) = h_ukf(X_sigma_pred(:, i));
    end
    
    % Hitung mean pengukuran yang diprediksi (y_pred)
    y_pred = Y_sigma_pred * Wm;
    
    % Hitung Inovasi Kovariansi (Pyy) dan Kovariansi Silang (Pxy)
    Pyy = R_ukf; 
    Pxy = zeros(nx, ny);
    
    for i = 1:2*nx+1
        Pyy = Pyy + Wc(i) * (Y_sigma_pred(:, i) - y_pred) * (Y_sigma_pred(:, i) - y_pred)';
        Pxy = Pxy + Wc(i) * (X_sigma_pred(:, i) - x_pred) * (Y_sigma_pred(:, i) - y_pred)';
    end
    
    % Hitung Kalman Gain (K)
    K = Pxy / Pyy;
    
    % Update State Estimasi (x_ukf) dan Kovariansi (P_ukf)
    inovasi = y_measured - y_pred;
    x_ukf = x_pred + K * inovasi;
    P_ukf = P_pred - K * Pyy * K';

    % Simpan hasil estimasi UKF
    history_x_ukf(:, k) = x_ukf;
end

% --- 6. Visualisasi Hasil Estimasi Angin ---
figure;
subplot(3,1,1);
plot(dt*(1:N_steps), history_x_ukf(7,:), 'b', 'LineWidth', 1.5);
hold on;
plot(dt*(1:N_steps), history_wind_actual(1,:), 'r--', 'LineWidth', 1.5);
title('Estimasi Gangguan Angin Wx');
xlabel('Waktu (s)');
ylabel('Kecepatan (m/s)');
legend('UKF Estimasi Wx', 'Angin Aktual Wx', 'Location', 'southeast');
grid on;

subplot(3,1,2);
plot(dt*(1:N_steps), history_x_ukf(8,:), 'b', 'LineWidth', 1.5);
hold on;
plot(dt*(1:N_steps), history_wind_actual(2,:), 'r--', 'LineWidth', 1.5);
title('Estimasi Gangguan Angin Wy');
xlabel('Waktu (s)');
ylabel('Kecepatan (m/s)');
legend('UKF Estimasi Wy', 'Angin Aktual Wy', 'Location', 'southeast');
grid on;

subplot(3,1,3);
plot(dt*(1:N_steps), history_x_ukf(9,:), 'b', 'LineWidth', 1.5);
hold on;
plot(dt*(1:N_steps), history_wind_actual(3,:), 'r--', 'LineWidth', 1.5);
title('Estimasi Gangguan Angin Wz');
xlabel('Waktu (s)');
ylabel('Kecepatan (m/s)');
legend('UKF Estimasi Wz', 'Angin Aktual Wz', 'Location', 'southeast');
grid on;