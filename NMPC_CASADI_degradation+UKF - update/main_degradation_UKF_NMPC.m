%% Adaptive NMPC with UKF for Payload Estimation (3D Quadrotor)
% This script combines a Unscented Kalman Filter (UKF) for payload (mass)
% estimation with a Nonlinear Model Predictive Control (NMPC) for 3D quadrotor
% trajectory tracking. The NMPC adapts its model based on the mass estimated by the UKF.

clear all;
close all;
clc;

% Add CasADi path (CHANGE THIS TO YOUR CASADI INSTALLATION PATH)
addpath('C:\Users\DELL\Documents\MATLAB\casadi-3.7.0-windows64-matlab2018b');
import casadi.*


%% --- 1. Definisi Parameter Sistem ---
% Parameter Drone Fisik
nominal_mass_value = 1.0; % kg (Massa nominal yang diketahui NMPC di awal)
true_grav = 9.81; % m/s^2
l = 0.25;   % Panjang lengan (m)
Ixx = 4.85e-3; % Momen inersia
Iyy = 4.85e-3;
Izz = 8.81e-3;

% Skenario Perubahan Payload (untuk simulasi "dunia nyata")
actual_mass_at_start = nominal_mass_value;
added_payload_mass = 1.0; % kg (payload tambahan)
time_of_payload_change = 6.0; % detik (waktu penambahan payload)
actual_changed_mass = actual_mass_at_start + added_payload_mass;

% Dimensi State dan Input (Quadrotor 3D)
nx = 12; % [px;py;pz;phi;theta;psi;vx;vy;vz;p;q;r]
nu = 4;  % [u1;u2;u3;u4] (Individual motor thrusts)
nx_ukf = nx + 1; % [px;py;pz;phi;theta;psi;vx;vy;vz;p;q;r;mass_est]

% Horizon NMPC
N_nmpc = 20; % Langkah waktu prediksi NMPC
dt_nmpc = 0.05; % Ukuran langkah waktu NMPC

% UKF
dt_ukf = 0.01; % Ukuran langkah waktu UKF (lebih cepat dari NMPC untuk estimasi lebih halus)

% Waktu Simulasi
T_sim = 10; % Total waktu simulasi (detik)
N_sim_total = T_sim / dt_ukf; % Jumlah langkah simulasi total

eta = [1.0; 1.0; 1.0; 1.0]; % Efisiensi awal semua motor 100%

%% --- 2. UKF Implementation ---

% Inisialisasi UKF
% State Awal UKF: [px;py;pz;phi;theta;psi;vx;vy;vz;p;q;r;mass_est]
ukf_x_est = zeros(nx_ukf, 1);
ukf_x_est(3) = 0.1; % Small initial Z position
ukf_x_est(end) = nominal_mass_value + 0; % Initial mass guess (slightly off)

% Kovariansi State Awal P (ukuran nx_ukf x nx_ukf)
% Memberikan ketidakpastian tinggi pada estimasi awal massa
ukf_P = diag([0.1, 0.1, 0.2, ... % px, py, pz
              0.01, 0.01, 0.01, ... % phi, theta, psi
              0.1, 0.1, 0.2, ... % vx, vy, vz
              0.01, 0.01, 0.01, ... % p, q, r
              0.5]); % mass (high uncertainty)

% Kovariansi Noise Proses Q (ukuran nx_ukf x nx_ukf)
% Noise pada dinamika, dan random walk pada massa (sangat kecil untuk massa)
ukf_Q = diag([0.001, 0.001, 0.000001, ... % px,py,pz
              0.0001, 0.0001, 0.0001, ... % phi,theta,psi
              0.01, 0.01, 0.0000001, ... % vx,vy,vz
              0.001, 0.001, 0.001, ... % p,q,r
              0.0001]); % mass (very small process noise for mass)

% Kovariansi Noise Pengukuran R (ukuran n_measurements x n_measurements)
% Mengukur posisi (px, py, pz) dan orientasi (phi, theta, psi)
ny_ukf = 6; % [px;py;pz;phi;theta;psi]
ukf_R = diag([0.05, 0.05, 0.00000001, ... % px, py, pz
              0.005, 0.005, 0.005]); % phi, theta, psi

% Parameter UKF (Unscented Transform)
alpha_ukf = 0.5; % Scaling parameter
kappa_ukf = 0;    % Secondary scaling parameter (sering 0 untuk state tinggi)
beta_ukf = 2;     % Parameter untuk distribusi Gaussian

% Fungsi Dinamika UKF (Continuous-time) - Memanggil helper function
% Input: x_ukf (13 states), u_applied (4 motor thrusts)
% Output: xdot_ukf (derivatives for each state)
ukf_dynamics_fcn_handle = @(x, u_applied, g_val, l_val, Ixx_val, Iyy_val, Izz_val) ...
    quadrotor_dynamics_for_ukf(x, u_applied, g_val, l_val, Ixx_val, Iyy_val, Izz_val);

% Fungsi Pengukuran UKF (Mengukur posisi dan orientasi)
% Input: x_ukf (13 states)
% Output: y_predicted (6 measurements)
ukf_measurement_fcn = @(x) x(1:6);

%% --- 3. NMPC Setup (CasADi Symbolic) ---

% --- Definisikan Model Dinamika Quadrotor Simbolik ---
x_sym = MX.sym('x', nx, 1); % State
u_sym = MX.sym('u', nu, 1); % Input (gaya dorong motor)
mass_sym_in_model  = MX.sym('mass_sym_in_model', 1); % Simbol untuk massa dalam model

% Konversi gaya dorong motor ke gaya total dan torsi
F_total = sum(u_sym);
tau_phi = l * (u_sym(2) - u_sym(4));
tau_theta = l * (u_sym(3) - u_sym(1));
tau_psi = 0.05 * (u_sym(1) - u_sym(2) + u_sym(3) - u_sym(4));

% Dinamika Quadrotor (extracted for readability)
px_s = x_sym(1); py_s = x_sym(2); pz_s = x_sym(3);
phi_s = x_sym(4); theta_s = x_sym(5); psi_s = x_sym(6);
vx_s = x_sym(7); vy_s = x_sym(8); vz_s = x_sym(9);
p_s = x_sym(10); q_s = x_sym(11); r_s = x_sym(12);

R_b_i = rotz(psi_s) * roty(theta_s) * rotx(phi_s);

accel_x = R_b_i(1,3) * F_total / mass_sym_in_model;
accel_y = R_b_i(2,3) * F_total / mass_sym_in_model;
accel_z = R_b_i(3,3) * F_total / mass_sym_in_model - true_grav;

p_dot = (tau_phi + (Iyy - Izz) * q_s * r_s) / Ixx;
q_dot = (tau_theta + (Izz - Ixx) * p_s * r_s) / Iyy;
r_dot = (tau_psi + (Ixx - Iyy) * p_s * q_s) / Izz;

xdot = [vx_s; vy_s; vz_s; ... % Posisi
        p_s + q_s*sin(phi_s)*tan(theta_s) + r_s*cos(phi_s)*tan(theta_s); ... % phi_dot
        q_s*cos(phi_s) - r_s*sin(phi_s); ... % theta_dot
        q_s*sin(phi_s)/cos(theta_s) + r_s*cos(phi_s)/cos(theta_s); ... % psi_dot
        accel_x; accel_y; accel_z; ... % Kecepatan Linear
        p_dot; q_dot; r_dot]; % Kecepatan Angular

% Fungsi CasADi untuk dinamika sistem (Continuous-time)
f_quadrotor = Function('f_quadrotor',{x_sym,u_sym,mass_sym_in_model},{xdot},{'x','u','mass'},{'xdot'});

% Diskretisasi Model (Menggunakan metode Runge-Kutta ke-4)
X_rk4 = MX.sym('X_rk4', nx, 1);
U_rk4 = MX.sym('U_rk4', nu, 1);
Mass_rk4 = MX.sym('Mass_rk4', 1);
XDOT_rk4 = f_quadrotor(X_rk4, U_rk4, Mass_rk4);
k1 = XDOT_rk4;
k2 = f_quadrotor(X_rk4 + dt_nmpc/2*k1, U_rk4, Mass_rk4);
k3 = f_quadrotor(X_rk4 + dt_nmpc/2*k2, U_rk4, Mass_rk4);
k4 = f_quadrotor(X_rk4 + dt_nmpc*k3, U_rk4, Mass_rk4);
X_next_rk4 = X_rk4 + dt_nmpc/6*(k1 + 2*k2 + 2*k3 + k4);
F_discrete_quadrotor = Function('F_discrete_quadrotor', {X_rk4, U_rk4, Mass_rk4}, {X_next_rk4});

% --- Rumuskan Masalah Optimasi NMPC (Multiple Shooting) ---
w = {}; % Variabel optimasi utama yang akan diumpankan ke solver
J = 0;  % Fungsi biaya
g = {}; % Kendala (kesetaraan dan ketidaksetaraan)

lbw = []; ubw = []; w0 = []; % Batasan dan tebakan awal untuk variabel optimasi
lbg = []; ubg = []; % Batasan untuk kendala

% Variabel state dan input untuk setiap langkah horizon
X_vars = cell(N_nmpc+1, 1); % X_0, X_1, ..., X_N
U_vars = cell(N_nmpc, 1);   % U_0, U_1, ..., U_{N-1}

% Parameter untuk solver NMPC: [Current_State (nx); Reference_State (nx); Estimated_Mass (1)]
n_params_nmpc = nx + nx + 1; 
all_params_nmpc_sym = MX.sym('all_params', n_params_nmpc, 1); 
X_initial_param = all_params_nmpc_sym(1:nx);             % Current state
X_ref_param = all_params_nmpc_sym(nx+1 : nx+nx);     % Reference state
Estimated_Mass_Param = all_params_nmpc_sym(nx+nx+1); % Estimated mass from UKF

% Langkah 0: Variabel State X_0
X_vars{1} = MX.sym('X_0', nx, 1);
w = {w{:}, X_vars{1}};
lbw = [lbw; -inf*ones(nx,1)]; % State bounds (generally open, rely on dynamics)
ubw = [ubw;  inf*ones(nx,1)];
w0 = [w0; zeros(nx,1)];

% Kendala: State awal horizon prediksi harus sama dengan state saat ini
g = {g{:}, X_vars{1} - X_initial_param};
lbg = [lbg; zeros(nx,1)];
ubg = [ubg; zeros(nx,1)];

% Loop untuk sisa horizon prediksi (U_k dan X_{k+1})
for k = 0:N_nmpc-1
    % Variabel Input U_k
    U_vars{k+1} = MX.sym(['U_' num2str(k)], nu, 1);
    w = {w{:}, U_vars{k+1}};
    lbw = [lbw; 0*ones(nu,1)];   % Batas bawah gaya dorong motor (min 0N)
    ubw = [ubw; 20*ones(nu,1)];  % Batas atas gaya dorong motor (misal, 20N)
    w0 = [w0; 0.25*true_grav*nominal_mass_value*ones(nu,1)]; % Tebakan awal untuk hovering

    % Variabel State X_{k+1}
    X_vars{k+2} = MX.sym(['X_' num2str(k+1)], nx, 1);
    w = {w{:}, X_vars{k+2}};
    lbw = [lbw; -inf*ones(nx,1)];
    ubw = [ubw;  inf*ones(nx,1)];
    w0 = [w0; zeros(nx,1)]; % Tebakan awal state

    % Fungsi biaya untuk langkah k
    Q_cost = diag([100, 100, 100, ... % px, py, pz
                   10, 10, 10, ...   % phi, theta, psi
                   1, 1, 1, ...      % vx, vy, vz
                   0.1, 0.1, 0.1]);  % p, q, r
    R_cost = diag([0.1, 0.1, 0.1, 0.1]); % Bobot upaya kontrol

    % Menggunakan parameter referensi dari input solver
    J = J + (X_vars{k+1} - X_ref_param)'*Q_cost*(X_vars{k+1} - X_ref_param) + ...
            U_vars{k+1}'*R_cost*U_vars{k+1};
    
    % Kendala dinamika (Multiple Shooting)
    % Gunakan F_discrete_quadrotor dengan estimated_mass_param
    g = {g{:}, F_discrete_quadrotor(X_vars{k+1}, U_vars{k+1}, Estimated_Mass_Param) - X_vars{k+2}};
    lbg = [lbg; zeros(nx,1)]; % Kendala kesetaraan (hasil harus 0)
    ubg = [ubg; zeros(nx,1)]; % Kendala kesetaraan (hasil harus 0)
end

% Terminal Cost (Optional): Penalti pada state akhir horizon
J = J + (X_vars{N_nmpc+1} - X_ref_param)'*Q_cost*(X_vars{N_nmpc+1} - X_ref_param);

% Gabungkan semua variabel, fungsi biaya, dan kendala
nlp = struct('f', J, 'x', vertcat(w{:}), 'g', vertcat(g{:}), 'p', all_params_nmpc_sym);

% Konfigurasi solver IPOPT
solver_opts = struct;
solver_opts.print_time = false;
solver_opts.ipopt.max_iter = 100;
solver_opts.ipopt.tol = 1e-6;
solver_opts.ipopt.linear_solver = 'mumps'; 
solver_opts.ipopt.hessian_approximation = 'limited-memory'; 
solver_nmpc = nlpsol('solver_nmpc', 'ipopt', nlp, solver_opts);

%% --- 4. Main Simulation Loop ---

% Inisialisasi data log
history_x_actual = zeros(nx, N_sim_total + 1);
history_u_nmpc = zeros(nu, N_sim_total);
history_x_ref = zeros(nx, N_sim_total + 1);
history_ukf_x_est = zeros(nx_ukf, N_sim_total + 1);
history_actual_mass = zeros(1, N_sim_total + 1);

% State Awal "Dunia Nyata" (Actual Drone)
current_actual_x = zeros(nx, 1);
current_actual_x(3) = 0.0; % Start at Z=0
current_actual_mass = actual_mass_at_start;

history_x_actual(:, 1) = current_actual_x;
history_actual_mass(1) = current_actual_mass;

%history degradations
history_phi_actual = zeros(1, N_sim_total + 1);
history_theta_actual = zeros(1, N_sim_total + 1);
history_psi_actual = zeros(1, N_sim_total + 1);
history_p_actual = zeros(1, N_sim_total + 1);
history_q_actual = zeros(1, N_sim_total + 1);
history_r_actual = zeros(1, N_sim_total + 1);

% Simpan estimasi awal UKF
history_ukf_x_est(:, 1) = ukf_x_est;

% Tebakan awal NMPC (warm start)
arg_w0_nmpc = w0;

disp('Memulai Simulasi Adaptive NMPC dengan UKF (3D Quadrotor)...');
tic; % Start timer

% Loop Simulasi
for i = 1:N_sim_total
    current_time = (i-1) * dt_ukf;

    % --- 4.1. Perubahan Payload (Simulasi "Dunia Nyata") ---
    if current_time >= time_of_payload_change && current_actual_mass == actual_mass_at_start
        current_actual_mass = actual_changed_mass;
        fprintf('--- Perubahan Payload! Massa drone sekarang: %.2f kg pada t=%.2f s ---\n', current_actual_mass, current_time);
    end
    history_actual_mass(i+1) = current_actual_mass; % Log massa aktual

    % --- 4.2. UKF: Prediksi & Update ---
    % UKF mendapatkan input kontrol dari NMPC dari langkah sebelumnya.
    % Untuk langkah pertama, asumsikan u_applied_prev = nol
    if i == 1
        u_applied_prev = zeros(nu,1);
    else
        u_applied_prev = history_u_nmpc(:,i-1);
    end
    
    % Prediksi UKF (menggunakan model dinamika quadrotor full)
    [ukf_x_est_pred, ukf_P_pred] = ukf_predict(ukf_x_est, ukf_P, u_applied_prev, ukf_dynamics_fcn_handle, ukf_Q, dt_ukf, true_grav, alpha_ukf, kappa_ukf, beta_ukf, l, Ixx, Iyy, Izz);

    % Buat Pengukuran (z_actual, phi_actual, theta_actual, psi_actual dengan noise)
    measurement_y = ukf_measurement_fcn(current_actual_x) + randn(ny_ukf, 1) .* sqrt(diag(ukf_R));
    
    % Update UKF
    [ukf_x_est, ukf_P] = ukf_update1(ukf_x_est_pred, ukf_P_pred, measurement_y, ukf_measurement_fcn, ukf_R, alpha_ukf, kappa_ukf, beta_ukf);

    % Log estimasi UKF
    history_ukf_x_est(:, i+1) = ukf_x_est;

    % --- 4.3. NMPC: Hitung Input Kontrol (Adaptif) ---
    % NMPC dijalankan setiap dt_nmpc (bisa lebih jarang dari UKF)
    if mod(round(current_time/dt_nmpc)*dt_nmpc, dt_nmpc) < (dt_ukf/2) % Jalankan NMPC pada kelipatan dt_nmpc
        % State awal NMPC diambil dari estimasi UKF
        x0_for_nmpc = ukf_x_est(1:nx); % [px;py;pz;phi;theta;psi;vx;vy;vz;p;q;r]
        
        % Referensi NMPC (Spiral Trajectory)
        x_ref_for_nmpc = QuadrotorReferenceTrajectory5(current_time, nominal_mass_value);

        % Massa yang diestimasi untuk NMPC
        estimated_mass_for_nmpc = ukf_x_est(end); % Massa dari state UKF (terakhir)

        % Parameter untuk NMPC solver
        params_for_nmpc_solver = vertcat(x0_for_nmpc, x_ref_for_nmpc, estimated_mass_for_nmpc);

        % Panggil NMPC Solver
        sol_nmpc = solver_nmpc('x0', arg_w0_nmpc, ...
                                'lbx', lbw, 'ubx', ubw, ...
                                'lbg', lbg, 'ubg', ubg, ...
                                'p', params_for_nmpc_solver);
                      
        if current_time >= 6.0
            eta(1) = 0.5; % Misalnya, motor 1 rusak sebagian
        end
        
        u_optimal_nmpc = eta.*full(sol_nmpc.x(nx + 1 : nx + nu)); % Ambil U_0
        arg_w0_nmpc = shift_solution(full(sol_nmpc.x), nx, nu, N_nmpc);
        
        % Pastikan u_optimal_nmpc berada dalam batas (sekali lagi untuk keamanan)
        u_optimal_nmpc = max(0, min(20, u_optimal_nmpc)); % Clamp ulang ke batas [0, 20]
    else
        % Jika NMPC tidak dijalankan, gunakan input dari langkah NMPC sebelumnya
        u_optimal_nmpc = history_u_nmpc(:,i-1);
    end
    history_u_nmpc(:,i) = u_optimal_nmpc;
    history_x_ref(:,i) = QuadrotorReferenceTrajectory5(current_time, nominal_mass_value); % Log referensi

    % --- 4.4. Simulasi "Dunia Nyata" Drone (Aktual) ---
    % Gunakan input kontrol dari NMPC (u_optimal_nmpc) dan massa AKTUAL
    % Ini adalah model "true" yang mereplikasi perilaku fisik drone
    true_dynamics_fcn_handle = @(x_drone, u_val, g_val, l_val, Ixx_val, Iyy_val, Izz_val, m_val) ...
        quadrotor_dynamics_for_ukf(x_drone, u_val, g_val, l_val, Ixx_val, Iyy_val, Izz_val); % Note: mass is passed as x_ukf_current(13) in this fcn

    actual_x_for_dynamics = [current_actual_x; current_actual_mass]; % Create 13-state vector
    
    current_actual_x = rk4_step_quad(actual_x_for_dynamics, u_optimal_nmpc, ...
                                  dt_ukf, ukf_dynamics_fcn_handle, true_grav, l, Ixx, Iyy, Izz);
    current_actual_x = current_actual_x(1:nx); % Extract back to 12 states
    history_x_actual(:,i+1) = current_actual_x;
    
    %degub degradation
    history_phi_actual(i+1) = current_actual_x(4);
    history_theta_actual(i+1) = current_actual_x(5);
    history_psi_actual(i+1) = current_actual_x(6);

    history_p_actual(i+1) = current_actual_x(10);
    history_q_actual(i+1) = current_actual_x(11);
    history_r_actual(i+1) = current_actual_x(12);


    if mod(i, 100) == 0
        fprintf('Iterasi %d/%d, Waktu: %.2f s, Pz Aktual: %.2f m, Pz Est: %.2f m, Massa Est: %.2f kg\n', ...
                i, N_sim_total, current_time, current_actual_x(3), ukf_x_est(3), ukf_x_est(end));
    end
end
history_x_ref(:,N_sim_total+1) = QuadrotorReferenceTrajectory5(T_sim, nominal_mass_value); % Log referensi terakhir

toc; % Stop timer

%% --- 5. Plot Hasil ---
time_vec = 0:dt_ukf:T_sim;

figure('Name', 'Quadrotor Position Tracking');
subplot(3,1,1);
plot(time_vec, history_x_actual(1,:), 'b', 'LineWidth', 1.5); hold on;
plot(time_vec, history_ukf_x_est(1,:), 'g--', 'LineWidth', 1.0);
plot(time_vec, history_x_ref(1,:), 'r:', 'LineWidth', 1.0);
title('Quadrotor X Position');
xlabel('Time (s)'); ylabel('Position X (m)');
legend('Actual X', 'UKF Est X', 'Reference X');
grid on;

subplot(3,1,2);
plot(time_vec, history_x_actual(2,:), 'b', 'LineWidth', 1.5); hold on;
plot(time_vec, history_ukf_x_est(2,:), 'g--', 'LineWidth', 1.0);
plot(time_vec, history_x_ref(2,:), 'r:', 'LineWidth', 1.0);
title('Quadrotor Y Position');
xlabel('Time (s)'); ylabel('Position Y (m)');
legend('Actual Y', 'UKF Est Y', 'Reference Y');
grid on;

subplot(3,1,3);
plot(time_vec, history_x_actual(3,:), 'b', 'LineWidth', 1.5); hold on;
plot(time_vec, history_ukf_x_est(3,:), 'g--', 'LineWidth', 1.0);
plot(time_vec, history_x_ref(3,:), 'r:', 'LineWidth', 1.0);
title('Quadrotor Z Position');
xlabel('Time (s)'); ylabel('Position Z (m)');
legend('Actual Z', 'UKF Est Z', 'Reference Z');
grid on;

figure('Name', 'Quadrotor Velocity Tracking');
subplot(3,1,1);
plot(time_vec, history_x_actual(7,:), 'b', 'LineWidth', 1.5); hold on;
plot(time_vec, history_ukf_x_est(7,:), 'g--', 'LineWidth', 1.0);
plot(time_vec, history_x_ref(7,:), 'r:', 'LineWidth', 1.0);
title('Quadrotor X Velocity');
xlabel('Time (s)'); ylabel('Velocity X (m/s)');
legend('Actual VX', 'UKF Est VX', 'Reference VX');
grid on;

subplot(3,1,2);
plot(time_vec, history_x_actual(8,:), 'b', 'LineWidth', 1.5); hold on;
plot(time_vec, history_ukf_x_est(8,:), 'g--', 'LineWidth', 1.0);
plot(time_vec, history_x_ref(8,:), 'r:', 'LineWidth', 1.0);
title('Quadrotor Y Velocity');
xlabel('Time (s)'); ylabel('Velocity Y (m/s)');
legend('Actual VY', 'UKF Est VY', 'Reference VY');
grid on;

subplot(3,1,3);
plot(time_vec, history_x_actual(9,:), 'b', 'LineWidth', 1.5); hold on;
plot(time_vec, history_ukf_x_est(9,:), 'g--', 'LineWidth', 1.0);
plot(time_vec, history_x_ref(9,:), 'r:', 'LineWidth', 1.0);
title('Quadrotor Z Velocity');
xlabel('Time (s)'); ylabel('Velocity Z (m/s)');
legend('Actual VZ', 'UKF Est VZ', 'Reference VZ');
grid on;

figure('Name', 'Quadrotor Orientation Tracking');
subplot(3,1,1);
plot(time_vec, rad2deg(history_x_actual(4,:)), 'b', 'LineWidth', 1.5); hold on;
plot(time_vec, rad2deg(history_ukf_x_est(4,:)), 'g--', 'LineWidth', 1.0);
plot(time_vec, rad2deg(history_x_ref(4,:)), 'r:', 'LineWidth', 1.0);
title('Quadrotor Roll (Phi)');
xlabel('Time (s)'); ylabel('Roll (deg)');
legend('Actual Roll', 'UKF Est Roll', 'Reference Roll');
grid on;

subplot(3,1,2);
plot(time_vec, rad2deg(history_x_actual(5,:)), 'b', 'LineWidth', 1.5); hold on;
plot(time_vec, rad2deg(history_ukf_x_est(5,:)), 'g--', 'LineWidth', 1.0);
plot(time_vec, rad2deg(history_x_ref(5,:)), 'r:', 'LineWidth', 1.0);
title('Quadrotor Pitch (Theta)');
xlabel('Time (s)'); ylabel('Pitch (deg)');
legend('Actual Pitch', 'UKF Est Pitch', 'Reference Pitch');
grid on;

subplot(3,1,3);
plot(time_vec, rad2deg(history_x_actual(6,:)), 'b', 'LineWidth', 1.5); hold on;
plot(time_vec, rad2deg(history_ukf_x_est(6,:)), 'g--', 'LineWidth', 1.0);
plot(time_vec, rad2deg(history_x_ref(6,:)), 'r:', 'LineWidth', 1.0);
title('Quadrotor Yaw (Psi)');
xlabel('Time (s)'); ylabel('Yaw (deg)');
legend('Actual Yaw', 'UKF Est Yaw', 'Reference Yaw');
grid on;


figure('Name', 'Mass Estimation and Control Inputs');
subplot(2,1,1);
plot(time_vec, history_actual_mass, 'b', 'LineWidth', 1.5); hold on;
plot(time_vec, history_ukf_x_est(end,:), 'r--', 'LineWidth', 1.0);
title('Mass Actual vs. UKF Estimated');
xlabel('Time (s)'); ylabel('Mass (kg)');
legend('Actual Mass', 'UKF Estimated Mass');
grid on;

subplot(2,1,2);
plot(time_vec(1:end-1), history_u_nmpc(1,:), 'r', 'LineWidth', 1.0); hold on;
plot(time_vec(1:end-1), history_u_nmpc(2,:), 'g', 'LineWidth', 1.0);
plot(time_vec(1:end-1), history_u_nmpc(3,:), 'b', 'LineWidth', 1.0);
plot(time_vec(1:end-1), history_u_nmpc(4,:), 'k', 'LineWidth', 1.0);
title('NMPC Control Inputs (Motor Thrusts)');
xlabel('Time (s)'); ylabel('Thrust (N)');
legend('Motor 1', 'Motor 2', 'Motor 3', 'Motor 4');
grid on;

figure;
subplot(3,1,1); plot(time_vec, history_phi_actual); ylabel('\phi (rad)'); title('Roll Angle');
subplot(3,1,2); plot(time_vec, history_theta_actual); ylabel('\theta (rad)'); title('Pitch Angle');
subplot(3,1,3); plot(time_vec, history_psi_actual); ylabel('\psi (rad)'); title('Yaw Angle');
title('Angle');
xlabel('Time (s)');

figure;
subplot(3,1,1); plot(time_vec, history_p_actual); ylabel('p (rad/s)'); title('Angular Velocity p');
subplot(3,1,2); plot(time_vec, history_q_actual); ylabel('q (rad/s)'); title('Angular Velocity q');
subplot(3,1,3); plot(time_vec, history_r_actual); ylabel('r (rad/s)'); title('Angular Velocity r');
title('Angular velocity');
xlabel('Time (s)');


%% --- 6. 3D Visualization of Drone Trajectory ---
figure('Name', '3D Quadrotor Trajectory');
h_traj = plot3(history_x_actual(1,:), history_x_actual(2,:), history_x_actual(3,:), 'b-', 'LineWidth', 1.5);
hold on;
h_ref_traj = plot3(history_x_ref(1,:), history_x_ref(2,:), history_x_ref(3,:), 'r--', 'LineWidth', 1.0);
grid on;
axis equal;
xlabel('X (m)'); ylabel('Y (m)'); zlabel('Z (m)');
title('3D Quadrotor Trajectory (Actual vs Reference)');
legend('Actual Trajectory', 'Reference Trajectory');

% Draw a simple quadrotor body (adjust size as needed)
prop_len = 0.2; % Length of propeller arms
prop_width = 0.05; % Width of propellers
body_size = 0.1; % Size of central body
h_drone = []; % Handle for drone body elements

% Initial drone position for drawing
px_curr = history_x_actual(1,1);
py_curr = history_x_actual(2,1);
pz_curr = history_x_actual(3,1);
phi_curr = history_x_actual(4,1);
theta_curr = history_x_actual(5,1);
psi_curr = history_x_actual(6,1);

% Define drone body vertices relative to its center (body frame)
arm_verts = [
    prop_len, 0, 0; -prop_len, 0, 0; % X arm
    0, prop_len, 0; 0, -prop_len, 0; % Y arm
];

% Draw central body (as a small cube or sphere for simplicity)
[sx, sy, sz] = sphere(8);
sx = sx * body_size; sy = sy * body_size; sz = sz * body_size;
h_body = surf(sx, sy, sz, 'FaceColor', 'cyan', 'EdgeColor', 'none', 'FaceAlpha', 0.8);

% Draw arms
h_arm1 = line([-prop_len, prop_len], [0, 0], [0, 0], 'Color', 'black', 'LineWidth', 3);
h_arm2 = line([0, 0], [-prop_len, prop_len], [0, 0], 'Color', 'black', 'LineWidth', 3);

% Set initial drone pose
set(h_body, 'XData', sx + px_curr, 'YData', sy + py_curr, 'ZData', sz + pz_curr);
% Arms are harder to rotate directly. We'll use a transform.
h_arms = [h_arm1, h_arm2];

% Use a transform object for the drone body to easily rotate/translate
h_drone_group = hgtransform;
set([h_body, h_arm1, h_arm2], 'Parent', h_drone_group);

% Animate the drone
view_angle = 30; % Initial view angle for better visualization
camorbit(view_angle, 0);

% If you want to animate the drone moving (this can be slow for many steps)
% Uncomment the following block:
for k_anim = 1:5:N_sim_total % Animate every 5 steps
    px_curr = history_x_actual(1,k_anim);
    py_curr = history_x_actual(2,k_anim);
    pz_curr = history_x_actual(3,k_anim);
    phi_curr = history_x_actual(4,k_anim);
    theta_curr = history_x_actual(5,k_anim);
    psi_curr = history_x_actual(6,k_anim);

    % Create transformation matrices
    T_pos = makehgtform('translate', px_curr, py_curr, pz_curr);
    R_orient = makehgtform('zrotate', psi_curr, 'yrotate', theta_curr, 'xrotate', phi_curr);
    
    % Apply the transformation
    set(h_drone_group, 'Matrix', T_pos * R_orient);
    
    drawnow limitrate;
    % pause(0.01); % Optional: slow down animation
end

%% --- 0. Global Helper Functions (for UKF dynamics and rotations) ---
% These functions are needed by the UKF dynamics model which is not CasADi symbolic
% and for plotting the drone.

function xdot = quadrotor_dynamics_for_ukf(x_ukf_current, u_applied, g_val, l_val, Ixx_val, Iyy_val, Izz_val)
    % Extracts states from the UKF's state vector (which includes mass)
    px = x_ukf_current(1); py = x_ukf_current(2); pz = x_ukf_current(3);
    phi = x_ukf_current(4); theta = x_ukf_current(5); psi = x_ukf_current(6);
    vx = x_ukf_current(7); vy = x_ukf_current(8); vz = x_ukf_current(9);
    p = x_ukf_current(10); q = x_ukf_current(11); r = x_ukf_current(12);
    estimated_mass = x_ukf_current(13); % This is the estimated mass from UKF state

    % Extract motor thrusts (u_applied is a 4x1 vector for individual motors)
    u1 = u_applied(1); u2 = u_applied(2); u3 = u_applied(3); u4 = u_applied(4);

    % Convert individual motor thrusts to total force and torques
    F_total = u1 + u2 + u3 + u4;
    tau_phi = l_val * (u2 - u4);
    tau_theta = l_val * (u3 - u1);
    tau_psi = 0.05 * (u1 - u2 + u3 - u4); % Assume 0.05 is d_prop constant

    % Rotation matrix from Body Frame to Inertial Frame
    R_b_i = rotz(psi) * roty(theta) * rotx(phi);

    % Linear accelerations
    accel_x = R_b_i(1,3) * F_total / estimated_mass;
    accel_y = R_b_i(2,3) * F_total / estimated_mass;
    accel_z = R_b_i(3,3) * F_total / estimated_mass - g_val;

    % Angular accelerations (Euler equations)
    p_dot = (tau_phi + (Iyy_val - Izz_val) * q * r) / Ixx_val;
    q_dot = (tau_theta + (Izz_val - Ixx_val) * p * r) / Iyy_val;
    r_dot = (tau_psi + (Ixx_val - Iyy_val) * p * q) / Izz_val;

    % State derivative (dx/dt)
    xdot = [vx; vy; vz; ... % Position derivatives
            p + q*sin(phi)*tan(theta) + r*cos(phi)*tan(theta); ... % phi_dot
            q*cos(phi) - r*sin(phi); ... % theta_dot
            q*sin(phi)/cos(theta) + r*cos(phi)/cos(theta); ... % psi_dot
            accel_x; accel_y; accel_z; ... % Linear velocities derivatives (accelerations)
            p_dot; q_dot; r_dot; ... % Angular velocities derivatives (angular accelerations)
            0]; % dm = 0 (mass is assumed constant in prediction step, updated by measurement)
end

% Rotation functions (used in dynamics)
function R_x = rotx(t)
    R_x = [1, 0, 0; 0, cos(t), -sin(t); 0, sin(t), cos(t)];
end
function R_y = roty(t)
    R_y = [cos(t), 0, sin(t); 0, 1, 0; -sin(t), 0, cos(t)];
end
function R_z = rotz(t)
    R_z = [cos(t), -sin(t), 0; sin(t), cos(t), 0; 0, 0, 1];
end

% RK4 Step for Actual Drone (Truth Model)
% RK4 Step for Actual Drone (Truth Model)
% This function now expects the 13-state vector (x_current including mass)
% but the dynamics_fcn internally extracts the mass.
function x_next = rk4_step_quad(x_current, u_applied, dt_val, dynamics_fcn, g_val, l_val, Ixx_val, Iyy_val, Izz_val)
    k1 = dynamics_fcn(x_current, u_applied, g_val, l_val, Ixx_val, Iyy_val, Izz_val);
    k2 = dynamics_fcn(x_current + dt_val/2*k1, u_applied, g_val, l_val, Ixx_val, Iyy_val, Izz_val);
    k3 = dynamics_fcn(x_current + dt_val/2*k2, u_applied, g_val, l_val, Ixx_val, Iyy_val, Izz_val);
    k4 = dynamics_fcn(x_current + dt_val*k3, u_applied, g_val, l_val, Ixx_val, Iyy_val, Izz_val);
    x_next = x_current + dt_val/6*(k1 + 2*k2 + 2*k3 + k4);
end

% --- UKF Predict Function (Generic Implementation) ---
% Assuming a generic UKF predict function
function [x_pred, P_pred] = ukf_predict(x_est, P, u, ukf_dynamics_fcn, Q, dt_ukf, g_val, alpha, kappa, beta, l_val, Ixx_val, Iyy_val, Izz_val)
    nx_ukf = numel(x_est);
    lambda = alpha^2 * (nx_ukf + kappa) - nx_ukf;
    Wm = [lambda / (nx_ukf + lambda); 0.5 / (nx_ukf + lambda) * ones(2*nx_ukf, 1)];
    Wc = Wm;
    Wc(1) = Wc(1) + (1 - alpha^2 + beta);

    % Ensure P is symmetric and positive definite before Cholesky
    P = (P + P') / 2;
    P = P + eye(size(P)) * 1e-7; % <-- Changed from 1e-9 to 1e-7

    sqrtP = chol(P, 'lower');
    X_sigma = repmat(x_est, 1, 2*nx_ukf+1);
    X_sigma(:, 2:nx_ukf+1) = X_sigma(:, 2:nx_ukf+1) + sqrt(nx_ukf + lambda) * sqrtP;
    X_sigma(:, nx_ukf+2:end) = X_sigma(:, nx_ukf+2:end) - sqrt(nx_ukf + lambda) * sqrtP;
    
    X_sigma_pred = zeros(nx_ukf, 2*nx_ukf+1);
    for i = 1:2*nx_ukf+1
        % Propagate sigma points through nonlinear dynamics
        % Use RK4 for integration of the continuous-time dynamics
        k1 = ukf_dynamics_fcn(X_sigma(:, i), u, g_val, l_val, Ixx_val, Iyy_val, Izz_val);
        k2 = ukf_dynamics_fcn(X_sigma(:, i) + dt_ukf/2*k1, u, g_val, l_val, Ixx_val, Iyy_val, Izz_val);
        k3 = ukf_dynamics_fcn(X_sigma(:, i) + dt_ukf/2*k2, u, g_val, l_val, Ixx_val, Iyy_val, Izz_val);
        k4 = ukf_dynamics_fcn(X_sigma(:, i) + dt_ukf*k3, u, g_val, l_val, Ixx_val, Iyy_val, Izz_val);
        X_sigma_pred(:, i) = X_sigma(:, i) + dt_ukf/6*(k1 + 2*k2 + 2*k3 + k4);
    end
    
    x_pred = X_sigma_pred * Wm;
    P_pred = Q; % Additive noise
    for i = 1:2*nx_ukf+1
        P_pred = P_pred + Wc(i) * (X_sigma_pred(:, i) - x_pred) * (X_sigma_pred(:, i) - x_pred)';
    end
    % Ensure P_pred is symmetric and positive definite
    P_pred = (P_pred + P_pred') / 2;
    P_pred = P_pred + eye(size(P_pred)) * 1e-7; % <-- Changed from 1e-9 to 1e-7
end

% --- UKF Update Function (Generic Implementation) ---
% Assuming a generic UKF update function
function [x_est, P_est] = ukf_update(x_pred, P_pred, y_meas, ukf_measurement_fcn, R, alpha, kappa, beta)
    nx_ukf = numel(x_pred);
    ny_ukf = numel(y_meas);
    lambda = alpha^2 * (nx_ukf + kappa) - nx_ukf;
    Wm = [lambda / (nx_ukf + lambda); 0.5 / (nx_ukf + lambda) * ones(2*nx_ukf, 1)];
    Wc = Wm;
    Wc(1) = Wc(1) + (1 - alpha^2 + beta);
    
    % Ensure P_pred is symmetric and positive definite before Cholesky
    P_pred = (P_pred + P_pred') / 2;
    P_pred = P_pred + eye(size(P_pred)) * 1e-7; % <-- Changed from 1e-9 to 1e-7

    sqrtP_pred = chol(P_pred, 'lower');
    X_sigma_pred_recalc = repmat(x_pred, 1, 2*nx_ukf+1);
    X_sigma_pred_recalc(:, 2:nx_ukf+1) = X_sigma_pred_recalc(:, 2:nx_ukf+1) + sqrt(nx_ukf + lambda) * sqrtP_pred;
    X_sigma_pred_recalc(:, nx_ukf+2:end) = X_sigma_pred_recalc(:, nx_ukf+2:end) - sqrt(nx_ukf + lambda) * sqrtP_pred;
    
    Y_sigma_pred = zeros(ny_ukf, 2*nx_ukf+1);
    for i = 1:2*nx_ukf+1
        Y_sigma_pred(:, i) = ukf_measurement_fcn(X_sigma_pred_recalc(:, i));
    end
    y_pred = Y_sigma_pred * Wm;
    
    Pyy = R; Pxy = zeros(nx_ukf, ny_ukf);
    for i = 1:2*nx_ukf+1
        Pyy = Pyy + Wc(i) * (Y_sigma_pred(:, i) - y_pred) * (Y_sigma_pred(:, i) - y_pred)';
        Pxy = Pxy + Wc(i) * (X_sigma_pred_recalc(:, i) - x_pred) * (Y_sigma_pred(:, i) - y_pred)';
    end
    
    % Ensure Pyy is symmetric and positive definite
    Pyy = (Pyy + Pyy') / 2;
    Pyy = Pyy + eye(size(Pyy)) * 1e-7; % <-- Changed from 1e-9 to 1e-7
    
    K = Pxy / Pyy; % Kalman Gain
    innovation = y_meas - y_pred;
    x_est = x_pred + K * innovation;
    P_est = P_pred - K * Pyy * K';
    % Ensure P_est is symmetric and positive definite
    P_est = (P_est + P_est') / 2;
    P_est = P_est + eye(size(P_est)) * 1e-7; % <-- Changed from 1e-9 to 1e-7
end

% Function to shift NMPC solution for warm start
function w_shifted = shift_solution(w_opt, nx, nu, N)
    extracted_x = cell(N+1, 1);
    extracted_u = cell(N, 1);
    offset = 0;
    extracted_x{1} = w_opt(offset + 1 : offset + nx);
    offset = offset + nx;
    for k = 0:N-1
        extracted_u{k+1} = w_opt(offset + 1 : offset + nu);
        offset = offset + nu;
        extracted_x{k+2} = w_opt(offset + 1 : offset + nx);
        offset = offset + nx;
    end
    
    w_shifted = [];
    w_shifted = [w_shifted; extracted_x{2}]; % X_1 becomes new X_0
    for k = 0:N-2
        w_shifted = [w_shifted; extracted_u{k+2}]; % U_{k+1} becomes new U_k
        w_shifted = [w_shifted; extracted_x{k+3}]; % X_{k+2} becomes new X_{k+1}
    end
    w_shifted = [w_shifted; extracted_u{N}]; % Last U is repeated
    w_shifted = [w_shifted; extracted_x{N+1}]; % Last X is repeated
end

