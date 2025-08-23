mm%% Adaptive NMPC dengan UKF untuk Estimasi Payload (Sumbu Z Saja)
% Ini adalah contoh sederhana untuk mengilustrasikan konsep.
% Dinamika drone disederhanakan hanya ke sumbu Z.

clear all;
close all;
clc;

addpath('C:\Users\DELL\Documents\MATLAB\casadi-3.7.0-windows64-matlab2018b'); % Ganti dengan path instalasi CasADi Anda
import casadi.*

%% 1. Definisi Parameter Sistem
% Parameter Drone (nominal dan aktual)
nominal_mass_value = 1.0; % kg (Massa nominal yang diketahui NMPC di awal)
true_grav = 9.81; % m/s^2

% Skenario Perubahan Payload (untuk simulasi "dunia nyata")
actual_mass_at_start = nominal_mass_value;
added_payload_mass = 1.0; % kg (payload tambahan)l
time_of_payload_change = 5.0; % detik (waktu penambahan payload)
actual_changed_mass = actual_mass_at_start + added_payload_mass;

% Dimensi State dan Input (Sumbu Z saja)
nx_drone = 2; % [z; vz]
nu_drone = 1; % [TotalThrust_z] (Gaya dorong vertikal)
nx_ukf = nx_drone + 1; % [z; vz; mass] -> State yang diestimasi UKF

% Horizon NMPC
N_nmpc = 10; % Langkah waktu prediksi NMPC
dt_nmpc = 0.1; % Ukuran langkah waktu NMPC

% UKF
dt_ukf = 0.05; % Ukuran langkah waktu UKF (bisa lebih cepat dari NMPC)

% Waktu Simulasi
T_sim = 10; % Total waktu simulasi (detik)
N_sim_total = T_sim / dt_ukf; % Jumlah langkah simulasi total

%% 2. Implementasi UKF Sederhana

% Inisialisasi UKF
% State Awal UKF: [z_est; vz_est; mass_est]
ukf_x_est = [0.1; 0.0; nominal_mass_value + 0.3]; % Tebakan awal z, vz, dan massa (sedikit salah)

% Kovariansi State Awal P (ukuran nx_ukf x nx_ukf)
ukf_P = diag([0.2, 0.2, 0.5]); % Ketidakpastian awal z, vz, dan massa (massa lebih tidak yakin)

% Kovariansi Noise Proses Q (ukuran nx_ukf x nx_ukf)
% Noise pada dinamika z, vz, dan random walk pada massa (sangat kecil untuk massa)
% ukf_Q = diag([0.0001, 0.0001, 0.0001]);
ukf_Q = diag([0.000001, 0.0000001, 0.0001]);
% ukf_Q = diag([1.00e-03, 1.00e-02, 1.00e-06]);

% Kovariansi Noise Pengukuran R (ukuran n_measurements x n_measurements)
% Misal hanya mengukur z
% ukf_R = diag([0.05]); % Noise pada pengukuran z
ukf_R = diag([0.00000001]); 

% Parameter UKF (untuk Unscented Transform)
alpha = 0.5; % Scaling parameter
kappa = 0;    % Secondary scaling parameter (sering 0 untuk state tinggi)
beta = 2;     % Parameter untuk distribusi Gaussian

% Fungsi Dinamika UKF (Continuous-time)
% Input: x_ukf (z, vz, m), u_applied (TotalThrust_z)
% Output: xdot_ukf (dz, dvz, dm)
ukf_dynamics_fcn = @(x, u_applied, g) ...
    [ x(2);                       % dz = vz
      (u_applied - x(3)*g) / x(3); % dvz = (Thrust - mass*g) / mass
      0 ];                        % dm = 0 (massa diasumsikan konstan)

% Fungsi Pengukuran UKF (Langsung dari state)
% Input: x_ukf (z, vz, m)
% Output: y_predicted (z)
% ukf_measurement_fcn = @(x) x(1); % Hanya mengukur z
ukf_measurement_fcn = @(x) x(1);

%% 3. Implementasi NMPC Sederhana (Adaptif)

% --- PENTING: Definisikan parameter NMPC di sini (di luar ODE struct) ---
% all_params_sym untuk NMPC solver secara keseluruhan
nmpc_params_sym = MX.sym('nmpc_p', nx_drone + nx_drone + 1); % [z0; vz0; z_ref; vz_ref; estimated_mass]
x0_nmpc_sym = nmpc_params_sym(1:nx_drone);
x_ref_nmpc_sym = nmpc_params_sym(nx_drone+1 : 2*nx_drone);
estimated_mass_nmpc_sym = nmpc_params_sym(2*nx_drone + 1); % Massa yang diestimasi dari UKF

% --- Setup Simbolik CasADi untuk NMPC ---
% States (simbolik)
z_nmpc_sym = MX.sym('z_nmpc', 1);
vz_nmpc_sym = MX.sym('vz_nmpc', 1);
x_nmpc_sym = vertcat(z_nmpc_sym, vz_nmpc_sym);

% Input (simbolik)
u_nmpc_sym = MX.sym('u_nmpc', 1); % Input kontrol untuk ODE

% Model Dinamika NMPC (menggunakan estimasi massa)
% nmpc_rhs = [x_nmpc_sym(2); (u_nmpc_sym - estimated_mass_nmpc_sym * true_grav) / estimated_mass_nmpc_sym]; % Sebelumnya
u_ode_sym = MX.sym('u_ode', nu_drone, 1); % Input kontrol untuk ODE
mass_ode_sym = MX.sym('mass_ode', 1);     % Massa untuk ODE

nmpc_rhs = [x_nmpc_sym(2); (u_ode_sym - mass_ode_sym * true_grav) / mass_ode_sym];

nmpc_ode = struct('x', x_nmpc_sym, ...       % State
                  'u', u_ode_sym, ...       % Input kontrol untuk ODE
                  'p', mass_ode_sym, ...    % Parameter model (massa) untuk ODE
                  'ode', nmpc_rhs);          % Ekspresi ODE

% Buat struct untuk opsi integrator
% Buat struct untuk opsi integrator
integrator_opts = struct;
integrator_opts.tf = dt_nmpc; % Set final time (duration of one step)
integrator_opts.expand = true;
integrator_opts.simplify = true;
% integrator_opts.rk_order = 4; % Ini hanya akan bekerja jika plugin 'rk' mendukungnya

F_nmpc_discrete = integrator('F_nmpc_discrete', 'rk', nmpc_ode, integrator_opts);

% --- Setup Optimasi NMPC ---
w = {}; % Variabel optimasi
J = 0;  % Fungsi biaya
g = {}; % Kendala
lbw = []; ubw = []; w0 = []; lbg = []; ubg = [];

X_vars_nmpc = cell(N_nmpc + 1, 1);
U_vars_nmpc = cell(N_nmpc, 1);

% State Awal (X_0)
X_vars_nmpc{1} = MX.sym('X0', nx_drone, 1);
w = {w{:}, X_vars_nmpc{1}};
lbw = [lbw; -inf; -inf]; ubw = [ubw; inf; inf]; w0 = [w0; 0; 0]; % Inisialisasi tebakan awal
g = {g{:}, X_vars_nmpc{1} - x0_nmpc_sym}; % Kendala: X_0 = x0_nmpc_sym
lbg = [lbg; zeros(nx_drone, 1)]; ubg = [ubg; zeros(nx_drone, 1)];

% Loop Horizon
for k = 0:N_nmpc-1
    % Input U_k
    U_vars_nmpc{k+1} = MX.sym(['U_' num2str(k)], nu_drone, 1);
    w = {w{:}, U_vars_nmpc{k+1}};
    lbw = [lbw; 0]; ubw = [ubw; 25]; % Batas gaya dorong (0N - 25N)
    w0 = [w0; 0];

    % State X_{k+1}
    X_vars_nmpc{k+2} = MX.sym(['X_' num2str(k+1)], nx_drone, 1);
    w = {w{:}, X_vars_nmpc{k+2}};
    lbw = [lbw; -inf; -inf]; ubw = [ubw; inf; inf]; w0 = [w0; 0; 0];

    % Kendala Dinamika
    current_x_nmpc = X_vars_nmpc{k+1};
    current_u_nmpc = U_vars_nmpc{k+1};
    % Ini adalah baris yang benar:
    predicted_result = F_nmpc_discrete('x0', current_x_nmpc, 'u', current_u_nmpc, 'p', estimated_mass_nmpc_sym);
    predicted_x_nmpc = predicted_result.xf; % <-- Akses state akhir dari output integrator
    g = {g{:}, predicted_x_nmpc - X_vars_nmpc{k+2}};
    lbg = [lbg; zeros(nx_drone, 1)]; ubg = [ubg; zeros(nx_drone, 1)];

    % Fungsi Biaya (Q untuk state error, R untuk control effort)
    Q_nmpc = diag([50, 10]); % Bobot z dan vz
    R_nmpc = diag([0.2]);    % Bobot gaya dorong
    J = J + (X_vars_nmpc{k+1} - x_ref_nmpc_sym)' * Q_nmpc * (X_vars_nmpc{k+1} - x_ref_nmpc_sym) + ...
            U_vars_nmpc{k+1}' * R_nmpc * U_vars_nmpc{k+1};

end
J = J + (X_vars_nmpc{N_nmpc+1} - x_ref_nmpc_sym)' * Q_nmpc * (X_vars_nmpc{N_nmpc+1} - x_ref_nmpc_sym);

% Setup Solver
nlp_nmpc = struct('f', J, 'x', vertcat(w{:}), 'g', vertcat(g{:}), 'p', nmpc_params_sym);
opts_nmpc = struct;
opts_nmpc.print_time = false;
opts_nmpc.ipopt.max_iter = 100;
opts_nmpc.ipopt.tol = 1e-6;
opts_nmpc.ipopt.linear_solver = 'mumps';
opts_nmpc.ipopt.hessian_approximation = 'limited-memory';
solver_nmpc = nlpsol('solver_nmpc', 'ipopt', nlp_nmpc, opts_nmpc);

%% 4. Loop Simulasi Utama

% Inisialisasi data log
history_z_actual = zeros(1, N_sim_total + 1);
history_vz_actual = zeros(1, N_sim_total + 1);
history_m_actual = zeros(1, N_sim_total + 1);
history_u_nmpc = zeros(1, N_sim_total);
history_z_ref = zeros(1, N_sim_total + 1);
history_z_ukf_est = zeros(1, N_sim_total + 1);
history_vz_ukf_est = zeros(1, N_sim_total + 1);
history_m_ukf_est = zeros(1, N_sim_total + 1);

% State Awal "Dunia Nyata" (Actual Drone)
current_actual_z = 0.0;
current_actual_vz = 0.0;
current_actual_mass = actual_mass_at_start;

history_z_actual(1) = current_actual_z;
history_vz_actual(1) = current_actual_vz;
history_m_actual(1) = current_actual_mass;

% Simpan estimasi awal UKF
history_z_ukf_est(1) = ukf_x_est(1);
history_vz_ukf_est(1) = ukf_x_est(2);
history_m_ukf_est(1) = ukf_x_est(3);

% Tebakan awal NMPC (warm start)
arg_w0_nmpc = w0;

% Target Trajectory (contoh: naik ke 2m, lalu diam)
z_ref_target = 2.0;

disp('Memulai Simulasi Adaptive NMPC dengan UKF...');
tic; % Start timer

% Loop Simulasi
for i = 1:N_sim_total
    current_time = (i-1) * dt_ukf;
    if current_time < 1.0
        estimated_mass_for_nmpc = nominal_mass_value;
    else
        estimated_mass_for_nmpc = ukf_x_est(3);
    end


    % --- 4.1. Perubahan Payload (Simulasi "Dunia Nyata") ---
    if current_time >= time_of_payload_change && current_actual_mass == actual_mass_at_start
        current_actual_mass = actual_changed_mass;
        fprintf('--- Perubahan Payload! Massa drone sekarang: %.2f kg pada t=%.2f s ---\n', current_actual_mass, current_time);
    end
    history_m_actual(i+1) = current_actual_mass; % Log massa aktual

    % --- 4.2. UKF: Prediksi & Update ---
    % UKF mendapatkan input kontrol dari NMPC dari langkah sebelumnya.
    % Untuk langkah pertama, asumsikan u_applied_prev = 0
    if i == 1
        u_applied_prev = 0;
    else
        u_applied_prev = history_u_nmpc(i-1);
    end
    
    % Prediksi UKF
    [ukf_x_est_pred, ukf_P_pred] = ukf_predict(ukf_x_est, ukf_P, u_applied_prev, ukf_dynamics_fcn, ukf_Q, dt_ukf, true_grav, alpha, kappa, beta);

    % Ukur z aktual dengan noise
    measurement_z = current_actual_z + randn * sqrt(ukf_R(1,1)); % Tambahkan noise
    
    % Update UKF
    [ukf_x_est, ukf_P] = ukf_update1(ukf_x_est_pred, ukf_P_pred, measurement_z, ukf_measurement_fcn, ukf_R, alpha, kappa, beta);

    % Log estimasi UKF
    history_z_ukf_est(i+1) = ukf_x_est(1);
    history_vz_ukf_est(i+1) = ukf_x_est(2);
    history_m_ukf_est(i+1) = ukf_x_est(3);

    % --- 4.3. NMPC: Hitung Input Kontrol (Adaptif) ---
    % NMPC dijalankan setiap dt_nmpc (misal lebih jarang dari UKF)
    if mod(current_time, dt_nmpc) < dt_ukf % Jalankan NMPC pada kelipatan dt_nmpc
        % State awal NMPC diambil dari estimasi UKF
        x0_for_nmpc = ukf_x_est(1:nx_drone); % [z_est; vz_est]
        
        % Referensi NMPC
        z_ref_current = z_ref_target; % Target konstan untuk contoh ini
        vz_ref_current = 0;
        x_ref_for_nmpc = [z_ref_current; vz_ref_current];

        % Massa yang diestimasi untuk NMPC
        estimated_mass_for_nmpc = ukf_x_est(3); % <--- MASSA DARI UKF KE NMPC!

        % Parameter untuk NMPC solver
        params_for_nmpc_solver = vertcat(x0_for_nmpc, x_ref_for_nmpc, estimated_mass_for_nmpc);

        % Panggil NMPC Solver
        sol_nmpc = solver_nmpc('x0', arg_w0_nmpc, ...
                                'lbx', lbw, 'ubx', ubw, ...
                                'lbg', lbg, 'ubg', ubg, ...
                                'p', params_for_nmpc_solver);
        
        u_optimal_nmpc = full(sol_nmpc.x(nx_drone + 1 : nx_drone + nu_drone)); % Ambil U_0
        arg_w0_nmpc = shift_solution(full(sol_nmpc.x), nx_drone, nu_drone, N_nmpc);
        
        % Pastikan u_optimal_nmpc berada dalam batas
        u_optimal_nmpc = max(0, min(25, u_optimal_nmpc)); % Clamp ulang untuk keamanan
    else
        % Jika NMPC tidak dijalankan, gunakan input dari langkah NMPC sebelumnya
        u_optimal_nmpc = history_u_nmpc(i-1);
    end
    history_u_nmpc(i) = u_optimal_nmpc;
    history_z_ref(i) = z_ref_target; % Log referensi

    % Gunakan input kontrol dari NMPC (u_optimal_nmpc) dan massa AKTUAL
    % Ini adalah model "true" yang mereplikasi perilaku fisik drone
    drone_ode_fcn = @(x_drone, u_val, m_val, g_val) [x_drone(2); (u_val - m_val*g_val) / m_val];

    % --- PERBAIKAN DI SINI ---
    % Tangkap output sebagai satu vektor
    actual_x_next = rk4_step([current_actual_z; current_actual_vz], u_optimal_nmpc, ...
                                                     dt_ukf, drone_ode_fcn, current_actual_mass, true_grav);

    % Kemudian ekstrak komponen-komponennya
    current_actual_z = actual_x_next(1);
    current_actual_vz = actual_x_next(2);

    history_z_actual(i+1) = current_actual_z;
    history_vz_actual(i+1) = current_actual_vz;
    if mod(i, 100) == 0
        fprintf('Iterasi %d/%d, Waktu: %.2f s, Z Aktual: %.2f m, Z Est: %.2f m, Massa Est: %.2f kg\n', ...
                i, N_sim_total, current_time, current_actual_z, ukf_x_est(1), ukf_x_est(3));
    end
end
history_z_ref(N_sim_total+1) = z_ref_target; % Log referensi terakhir

toc; % Stop timer

%% 5. Plot Hasil
time_vec = 0:dt_ukf:T_sim;

figure;
subplot(3,1,1);
plot(time_vec, history_z_actual, 'b', 'LineWidth', 1.5); hold on;
plot(time_vec, history_z_ukf_est, 'r--', 'LineWidth', 1.0);
plot(time_vec, history_z_ref, 'k:', 'LineWidth', 1.0);
title('Posisi Z Aktual vs. Estimasi UKF vs. Referensi');
xlabel('Waktu (s)'); ylabel('Posisi Z (m)');
legend('Aktual', 'UKF Estimasi', 'Referensi');
grid on;

subplot(3,1,2);
plot(time_vec, history_vz_actual, 'b', 'LineWidth', 1.5); hold on;
plot(time_vec, history_vz_ukf_est, 'r--', 'LineWidth', 1.0);
title('Kecepatan Z Aktual vs. Estimasi UKF');
xlabel('Waktu (s)'); ylabel('Kecepatan Z (m/s)');
legend('Aktual', 'UKF Estimasi');
grid on;

subplot(3,1,3);
plot(time_vec, history_m_actual, 'b', 'LineWidth', 1.5); hold on;
plot(time_vec, history_m_ukf_est, 'r--', 'LineWidth', 1.0);
title('Massa Aktual vs. Estimasi UKF');
xlabel('Waktu (s)'); ylabel('Massa (kg)');
legend('Aktual', 'UKF Estimasi');
grid on;

figure;
plot(time_vec(1:end-1), history_u_nmpc, 'm', 'LineWidth', 1.5);
title('Input Kontrol NMPC (Total Thrust Z)');
xlabel('Waktu (s)'); ylabel('Gaya Dorong (N)');
grid on;