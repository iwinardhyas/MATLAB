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
nx_ukf = nx + 1 + 4; % [px;py;pz;phi;theta;psi;vx;vy;vz;p;q;r;mass_est;eta1;eta2;eta3;eta4]

% Horizon NMPC
N_nmpc = 30; % Langkah waktu prediksi NMPC
dt_nmpc = 0.05; % Ukuran langkah waktu NMPC

% UKF
dt_ukf = 0.01; % Ukuran langkah waktu UKF (lebih cepat dari NMPC untuk estimasi lebih halus)

% Waktu Simulasi
T_sim = 5; % Total waktu simulasi (detik)
N_sim_total = T_sim / dt_ukf; % Jumlah langkah simulasi total

%% --- 2. UKF Implementation ---

% Inisialisasi UKF
% State Awal UKF: [px;py;pz;phi;theta;psi;vx;vy;vz;p;q;r;mass_est]
ukf_x_est = zeros(nx + 1 + 4, 1);  % [x; mass; eta1:eta4]
% ukf_x_est(1:nx) = initial_state_x;
ukf_x_est(nx+1) = nominal_mass_value; % misal 1.5
ukf_x_est(nx+2:nx+5) = ones(4,1);  % asumsi semua motor efisien 100%


% Kovariansi State Awal P (ukuran nx_ukf x nx_ukf)
% Memberikan ketidakpastian tinggi pada estimasi awal massa
ukf_P = diag([0.5, 0.5, 0.2, ... % px, py, pz
              0.01, 0.01, 0.01, ... % phi, theta, psi
              0.1, 0.1, 0.2, ... % vx, vy, vz
              0.01, 0.01, 0.01, ... % p, q, r
              0.01, ... % mass (high uncertainty)
              0.01, 0.01, 0.01, 0.01]); %degradation

% Kovariansi Noise Proses Q (ukuran nx_ukf x nx_ukf)
% Noise pada dinamika, dan random walk pada massa (sangat kecil untuk massa)
ukf_Q = diag([0.001, 0.001, 0.000001, ... % px,py,pz
              0.0001, 0.0001, 0.0001, ... % phi,theta,psi
              0.01, 0.01, 0.0000001, ... % vx,vy,vz
              0.001, 0.001, 0.001, ... % p,q,r
              0.00005, ... % mass (very small process noise for mass)
              0.005, 0.005, 0.005, 0.005]); %degradation

% Kovariansi Noise Pengukuran R (ukuran n_measurements x n_measurements)
% Mengukur posisi (px, py, pz) dan orientasi (phi, theta, psi)
ny_ukf = 6; % [px;py;pz;phi;theta;psi]
ukf_R = diag([0.05, 0.05, 0.05, ... % px, py, pz
              0.005, 0.005, 0.005]); % phi, theta, psi
% ukf_R = diag([
%     1e-3, 1e-3, 1e-3, ...       % vx, vy, vz
%     1e-4, 1e-4, 1e-4        % phi, theta, psi
% ]);


% Parameter UKF (Unscented Transform)
alpha_ukf = 0.02; % Scaling parameter
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
eta_sym = MX.sym('eta_sym', nu);
u_cmd_sym = MX.sym('u_cmd', nu, 1);

% Konversi gaya dorong motor ke gaya total dan torsi
% F_total = sum(u_sym);
% tau_phi = l * (u_sym(2) - u_sym(4));
% tau_theta = l * (u_sym(3) - u_sym(1));
% tau_psi = 0.05 * (u_sym(1) - u_sym(2) + u_sym(3) - u_sym(4));
% Dalam NMPC Setup (CasADi Symbolic):
F_total = sum(u_sym .* eta_sym); % <--- HARUSNYA SEPERTI INI
tau_phi = l * (u_sym(2) * eta_sym(2) - u_sym(4) * eta_sym(4)); % <--- HARUSNYA SEPERTI INI
tau_theta = l * (u_sym(3) * eta_sym(3) - u_sym(1) * eta_sym(1)); % <--- HARUSNYA SEPERTI INI
tau_psi = 0.05 * (u_sym(1) * eta_sym(1) - u_sym(2) * eta_sym(2) + u_sym(3) * eta_sym(3) - u_sym(4) * eta_sym(4)); % <--- HARUSNYA SEPERTI INI

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
f_quadrotor = Function('f_quadrotor',{x_sym,u_sym,mass_sym_in_model,eta_sym},{xdot},{'x','u','mass','eta_sym'},{'xdot'});

% Diskretisasi Model (Menggunakan metode Runge-Kutta ke-4)
X_rk4 = MX.sym('X_rk4', nx, 1);
U_rk4 = MX.sym('U_rk4', nu, 1);
Mass_rk4 = MX.sym('Mass_rk4', 1);
Eta_rk4 = MX.sym('Eta_rk4',nu);
XDOT_rk4 = f_quadrotor(X_rk4, U_rk4, Mass_rk4, Eta_rk4);
k1 = XDOT_rk4;
k2 = f_quadrotor(X_rk4 + dt_nmpc/2*k1, U_rk4, Mass_rk4,Eta_rk4);
k3 = f_quadrotor(X_rk4 + dt_nmpc/2*k2, U_rk4, Mass_rk4,Eta_rk4);
k4 = f_quadrotor(X_rk4 + dt_nmpc*k3, U_rk4, Mass_rk4,Eta_rk4);
X_next_rk4 = X_rk4 + dt_nmpc/6*(k1 + 2*k2 + 2*k3 + k4);
F_discrete_quadrotor = Function('F_discrete_quadrotor', {X_rk4, U_rk4, Mass_rk4, Eta_rk4}, {X_next_rk4});

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
n_params_nmpc = nx + nx + 5; 
all_params_nmpc_sym = MX.sym('all_params', n_params_nmpc, 1); 
X_initial_param = all_params_nmpc_sym(1:nx);             % Current state
X_ref_param = all_params_nmpc_sym(nx+1 : nx+nx);     % Reference state
Estimated_Mass_Param = all_params_nmpc_sym(nx+nx+1); % Estimated mass from UKF
Estimated_Eta_Param = all_params_nmpc_sym(nx+nx+2 : nx+nx+5);

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
%     U_vars{k+1} = MX.sym(['U_' num2str(k)], nu, 1);
    U_vars{k+1} = MX.sym(['U_' num2str(k)], nu, 1);
    w = {w{:}, U_vars{k+1}};
    lbw = [lbw; 0*ones(nu,1)];   % Batas bawah gaya dorong motor (min 0N)
    ubw = [ubw; 25*ones(nu,1)];  % Batas atas gaya dorong motor (misal, 20N)
    w0 = [w0; 0.25*true_grav*nominal_mass_value*ones(nu,1)]; % Tebakan awal untuk hovering

    % Variabel State X_{k+1}
    X_vars{k+2} = MX.sym(['X_' num2str(k+1)], nx, 1);
    w = {w{:}, X_vars{k+2}};
    lbw = [lbw; -inf*ones(nx,1)];
    ubw = [ubw;  inf*ones(nx,1)];
    w0 = [w0; zeros(nx,1)]; % Tebakan awal state

    % Fungsi biaya untuk langkah k
%     Q_cost = diag([100, 100, 100, ... % px, py, pz
%                    10, 10, 10, ...   % phi, theta, psi
%                    1, 1, 1, ...      % vx, vy, vz
%                    0.1, 0.1, 0.1]);  % p, q, r
    Q_cost = diag([
        100, ...   % px
        100, ...   % py
        200, ...   % pz
        300, ...   % phi (ϕ)
        300, ...  % theta (θ) - hasil tuning
        300, ...  % psi (ψ)   - hasil tuning
        50, ...    % vx
        50, ...    % vy
        50, ...    % vz
        20, ...    % p (angular rate)
        20, ...   % q (angular rate) - disarankan tinggi jika tracking θ penting
        20  ...   % r (angular rate) - disarankan tinggi jika tracking ψ penting
    ]);

    R_cost = diag([1, 1, 1, 1]); % Bobot upaya kontrol

    % Menggunakan parameter referensi dari input solver
    J = J + (X_vars{k+1} - X_ref_param)'*Q_cost*(X_vars{k+1} - X_ref_param) + ...
            U_vars{k+1}'*R_cost*U_vars{k+1};
    
    % Kendala dinamika (Multiple Shooting)
    % Gunakan F_discrete_quadrotor dengan estimated_mass_param
    g = {g{:}, F_discrete_quadrotor(X_vars{k+1}, U_vars{k+1}, Estimated_Mass_Param,Estimated_Eta_Param) - X_vars{k+2}};
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
solver_opts.ipopt.print_level = 0;
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

%degradasi actuator
current_eta_actual = ones(4, 1);
eta_fault_time = 3.0;


disp('Memulai Simulasi Adaptive NMPC dengan UKF (3D Quadrotor)...');
tic; % Start timer

% Loop Simulasi
for i = 1:N_sim_total
    current_time = (i-1) * dt_ukf;

    % --- 4.1. Perubahan Payload (Simulasi "Dunia Nyata") ---
%     if current_time >= time_of_payload_change && current_actual_mass == actual_mass_at_start
%         current_actual_mass = actual_changed_mass;
%         fprintf('--- Perubahan Payload! Massa drone sekarang: %.2f kg pada t=%.2f s ---\n', current_actual_mass, current_time);
%     end
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
        estimated_mass_for_nmpc = ukf_x_est(nx+1);  % Ambil elemen ke-13 untuk massa
        estimated_eta_for_nmpc = ukf_x_est(nx+2 : nx+5);     % Ambil elemen ke-14 untuk η


        % Parameter untuk NMPC solver
        params_for_nmpc_solver = vertcat(x0_for_nmpc, x_ref_for_nmpc, estimated_mass_for_nmpc,estimated_eta_for_nmpc);

        % Panggil NMPC Solver
        sol_nmpc = solver_nmpc('x0', arg_w0_nmpc, ...
                                'lbx', lbw, 'ubx', ubw, ...
                                'lbg', lbg, 'ubg', ubg, ...
                                'p', params_for_nmpc_solver);
                      
    if current_time >= eta_fault_time && all(current_eta_actual == 1.0)
        current_eta_actual = [0.7; 1.0; 1.0; 1.0];  % Hanya motor 1 yang rusak
        fprintf('--- Motor 1 rusak! Efisiensi saat ini: [%.2f %.2f %.2f %.2f] pada t = %.2f s ---\n', ...
            current_eta_actual(1), current_eta_actual(2), current_eta_actual(3), current_eta_actual(4), current_time);
    end


        
%         u_optimal_nmpc = estimated_eta_for_nmpc .* full(sol_nmpc.x(nx + 1 : nx + nu));
%         arg_w0_nmpc = shift_solution(full(sol_nmpc.x), nx, nu, N_nmpc);
        u_optimal_nmpc_raw = full(sol_nmpc.x(nx + 1 : nx + nu)); % Ini adalah nilai 'u_sym' dari optimasi
        arg_w0_nmpc = shift_solution(full(sol_nmpc.x), nx, nu, N_nmpc);
        
        % Pastikan u_optimal_nmpc berada dalam batas (sekali lagi untuk keamanan)
        u_optimal_nmpc = max(0, min(25, u_optimal_nmpc_raw)); % Clamp ulang ke batas [0, 20]
    else
        % Jika NMPC tidak dijalankan, gunakan input dari langkah NMPC sebelumnya
        u_optimal_nmpc = history_u_nmpc(:,i-1);
    end
    history_u_nmpc(:,i) = u_optimal_nmpc;
    history_x_ref(:,i) = QuadrotorReferenceTrajectory5(current_time, nominal_mass_value); % Log referensi
    
    u_degraded = current_eta_actual .* u_optimal_nmpc;

    % --- 4.4. Simulasi "Dunia Nyata" Drone (Aktual) ---
    % Gunakan input kontrol dari NMPC (u_optimal_nmpc) dan massa AKTUAL
    % Ini adalah model "true" yang mereplikasi perilaku fisik drone
    true_dynamics_fcn_handle = @(x,u,g,l,Ixx,Iyy,Izz) ...
        quadrotor_dynamics_for_ukf(x,u,g,l,Ixx,Iyy,Izz);

    actual_x_for_dynamics = [current_actual_x; current_actual_mass; current_eta_actual(:)];
    
    disp("Size actual_x_for_dynamics: " + num2str(size(actual_x_for_dynamics)));

    current_actual_x_full = rk4_step_quad(actual_x_for_dynamics, u_degraded, ...
                              dt_ukf, true_dynamics_fcn_handle, true_grav, l, Ixx, Iyy, Izz);
    current_actual_x = current_actual_x_full(1:nx); % Ambil 12 state pertama



    history_x_actual(:, i+1) = current_actual_x(1:12);

    
    %debug degradation
    history_phi_actual(i+1) = current_actual_x(4);
    history_theta_actual(i+1) = current_actual_x(5);
    history_psi_actual(i+1) = current_actual_x(6);

    history_p_actual(i+1) = current_actual_x(10);
    history_q_actual(i+1) = current_actual_x(11);
    history_r_actual(i+1) = current_actual_x(12);
    
        % Asumsikan Anda telah memperbaiki penunjukan index untuk massa dan eta di UKF_x_est
    % Misalnya: ukf_x_est(nx+1) untuk massa dan ukf_x_est(nx+2) untuk eta

    fprintf('Iterasi %d/%d, Waktu: %.2f s, ', ...
                 i, N_sim_total, current_time);
    fprintf('Px_Akt: %.2f, Py_Akt: %.2f, Pz_Akt: %.2f m, ', ...
                 current_actual_x(1), current_actual_x(2), current_actual_x(3));
    fprintf('Px_Est: %.2f, Py_Est: %.2f, Pz_Est: %.2f m, ', ...
                 ukf_x_est(1), ukf_x_est(2), ukf_x_est(3));
    fprintf('Massa_Est: %.2f kg, Eta_Est: %.4f\n', ...
                 ukf_x_est(nx+1), ukf_x_est(nx+2)); % <--- Asumsi nx+1 adalah massa, nx+2 adalah eta
    
    if mod(i, 100) == 0
        fprintf('Iterasi %d/%d, Waktu: %.2f s, Pz Aktual: %.2f m, Pz Est: %.2f m, Massa Est: %.2f kg\n', ...
            i, N_sim_total, current_time, current_actual_x(3), ukf_x_est(3), ukf_x_est(end-1));
    end
    
    if history_x_actual(3,:) <= 0
        break;
    end
end
history_x_ref(:,N_sim_total+1) = QuadrotorReferenceTrajectory5(T_sim, nominal_mass_value); % Log referensi terakhir

toc; % Stop timer

%% --- 5. Plot Hasil ---
time_vec = 0:dt_ukf:T_sim;

%% --- Plotting Results ---

figure;
subplot(3,1,1);
plot(0:dt_ukf:T_sim, history_ukf_x_est(nx+1,:), 'b', 'DisplayName', 'UKF Mass Est.');
hold on;
plot(0:dt_ukf:T_sim, history_actual_mass, 'r--', 'DisplayName', 'Actual Mass');
xlabel('Time (s)');
ylabel('Mass (kg)');
title('UKF Mass Estimation vs. Actual Mass');
legend('Location', 'best');
grid on;

subplot(3,1,2);
plot(0:dt_ukf:T_sim, history_ukf_x_est(nx+2,:), 'b', 'DisplayName', 'UKF Eta 1 Est.');
hold on;
plot(0:dt_ukf:T_sim, history_ukf_x_est(nx+3,:), 'g', 'DisplayName', 'UKF Eta 2 Est.');
plot(0:dt_ukf:T_sim, history_ukf_x_est(nx+4,:), 'm', 'DisplayName', 'UKF Eta 3 Est.');
plot(0:dt_ukf:T_sim, history_ukf_x_est(nx+5,:), 'c', 'DisplayName', 'UKF Eta 4 Est.');
% Plotting actual eta (ini perlu disesuaikan jika eta_actual berubah seiring waktu)
plot([0 eta_fault_time-dt_ukf], [1 1], 'k--', 'DisplayName', 'Actual Eta Pre-Fault');
plot([eta_fault_time eta_fault_time], [0 1.1], 'k:', 'DisplayName', 'Fault Time'); % Garis vertikal
plot([eta_fault_time T_sim], [0.7 0.7], 'r--', 'DisplayName', 'Actual Eta 1 Post-Fault');
plot([eta_fault_time T_sim], [1 1], 'g--', 'DisplayName', 'Actual Eta 2-4 Post-Fault');
xlabel('Time (s)');
ylabel('Actuator Efficiency (\eta)');
title('UKF Actuator Efficiency Estimation vs. Actual');
legend('Location', 'best');
grid on;

subplot(3,1,3);
plot(0:dt_ukf:T_sim, history_x_actual(3,:), 'b', 'DisplayName', 'Actual Z');
hold on;
plot(0:dt_ukf:T_sim, history_x_ref(3,:), 'r--', 'DisplayName', 'Ref Z');
xlabel('Time (s)');
ylabel('Z Position (m)');
title('Z Position Tracking');
legend('Location', 'best');
grid on;


figure;
subplot(3,1,1);
plot(0:dt_ukf:(N_sim_total-1)*dt_ukf, history_u_nmpc(1,:), 'DisplayName', 'NMPC u1');
hold on;
plot(0:dt_ukf:(N_sim_total-1)*dt_ukf, history_u_nmpc(2,:), 'DisplayName', 'NMPC u2');
plot(0:dt_ukf:(N_sim_total-1)*dt_ukf, history_u_nmpc(3,:), 'DisplayName', 'NMPC u3');
plot(0:dt_ukf:(N_sim_total-1)*dt_ukf, history_u_nmpc(4,:), 'DisplayName', 'NMPC u4');
xlabel('Time (s)');
ylabel('Requested Thrust (N)');
title('NMPC Requested Motor Thrusts');
legend('Location', 'best');
grid on;

subplot(3,1,2);
% Ini adalah input yang benar-benar diterapkan pada drone (sudah terdegradasi)
% Anda perlu menyimpan ini selama simulasi jika ingin diplot secara langsung.
% Untuk sekarang, saya akan menghitungnya ulang dari history_u_nmpc dan current_eta_actual
u_degraded_log = zeros(nu, N_sim_total);
for k = 1:N_sim_total
    time_k = (k-1) * dt_ukf;
    temp_eta = ones(4,1);
    if time_k >= eta_fault_time
        temp_eta = [0.7; 1.0; 1.0; 1.0];
    end
    u_degraded_log(:,k) = temp_eta .* history_u_nmpc(:,k);
end
plot(0:dt_ukf:(N_sim_total-1)*dt_ukf, u_degraded_log(1,:), 'DisplayName', 'Actual u1 (Degraded)');
hold on;
plot(0:dt_ukf:(N_sim_total-1)*dt_ukf, u_degraded_log(2,:), 'DisplayName', 'Actual u2');
plot(0:dt_ukf:(N_sim_total-1)*dt_ukf, u_degraded_log(3,:), 'DisplayName', 'Actual u3');
plot(0:dt_ukf:(N_sim_total-1)*dt_ukf, u_degraded_log(4,:), 'DisplayName', 'Actual u4');
xlabel('Time (s)');
ylabel('Actual Thrust Applied (N)');
title('Actual Motor Thrusts Applied (with Degradation)');
legend('Location', 'best');
grid on;

subplot(3,1,3);
plot(0:dt_ukf:T_sim, history_x_actual(4,:), 'DisplayName', 'Actual Phi');
hold on;
plot(0:dt_ukf:T_sim, history_x_actual(5,:), 'DisplayName', 'Actual Theta');
plot(0:dt_ukf:T_sim, history_x_actual(6,:), 'DisplayName', 'Actual Psi');
xlabel('Time (s)');
ylabel('Orientation (rad)');
title('Drone Orientation (Roll, Pitch, Yaw)');
legend('Location', 'best');
grid on;

% Tambahan: Plot kecepatan angular (p,q,r)
figure;
plot(0:dt_ukf:T_sim, history_x_actual(10,:), 'DisplayName', 'Actual p (roll rate)');
hold on;
plot(0:dt_ukf:T_sim, history_x_actual(11,:), 'DisplayName', 'Actual q (pitch rate)');
plot(0:dt_ukf:T_sim, history_x_actual(12,:), 'DisplayName', 'Actual r (yaw rate)');
xlabel('Time (s)');
ylabel('Angular Rate (rad/s)');
title('Drone Angular Rates');
legend('Location', 'best');
grid on;



%% --- 6. 3D Visualization of Drone Trajectory ---
figure('Name', '3D Quadrotor Trajectory');
h_traj = plot3(history_x_actual(1,:), history_x_actual(2,:), history_x_actual(3,:), 'b-', 'LineWidth', 1.5);
hold on;
h_ref_traj = plot3(history_x_ref(1,:)+10, history_x_ref(2,:)+10, history_x_ref(3,:)+10, 'r--', 'LineWidth', 1.0);
grid on;
axis equal;
xlabel('X (m)'); ylabel('Y (m)'); zlabel('Z (m)');
title('3D Quadrotor Trajectory (Actual vs Reference)');
legend('Actual Trajectory', 'Reference Trajectory');

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
    eta_sym = x_ukf_current(end-3:end);  % aman, 14:17 jika nx = 12


    % Extract motor thrusts (u_applied is a 4x1 vector for individual motors)
    u1 = u_applied(1); u2 = u_applied(2); u3 = u_applied(3); u4 = u_applied(4);

    % Convert individual motor thrusts to total force and torques
%     u_effective = eta .* u_applied;
%     F_total = sum(u_effective);
%     tau_phi = l_val * (u_effective(2) - u_effective(4));
%     tau_theta = l_val * (u_effective(3) - u_effective(1));
%     tau_psi = 0.05 * (u_effective(1) - u_effective(2) + u_effective(3) - u_effective(4));

    % Konversi gaya dorong motor ke gaya total dan torsi (dengan ETA)
    F_total = sum(u_applied .* eta_sym); % <--- Ubah ini!
    tau_phi = l_val * (u_applied(2) * eta_sym(2) - u_applied(4) * eta_sym(4)); % <--- Ubah ini!
    tau_theta = l_val * (u_applied(3) * eta_sym(3) - u_applied(1) * eta_sym(1)); % <--- Ubah ini!
    tau_psi = 0.05 * (u_applied(1) * eta_sym(1) - u_applied(2) * eta_sym(2) + u_applied(3) * eta_sym(3) - u_applied(4) * eta_sym(4)); % <--- Ubah ini!


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
    xdot = zeros(length(x_ukf_current), 1);
    xdot(1:3)   = [vx; vy; vz];
    xdot(4)     = p + q*sin(phi)*tan(theta) + r*cos(phi)*tan(theta);
    xdot(5)     = q*cos(phi) - r*sin(phi);
    xdot(6)     = q*sin(phi)/cos(theta) + r*cos(phi)/cos(theta);
    xdot(7:9)   = [accel_x; accel_y; accel_z];
    xdot(10:12) = [p_dot; q_dot; r_dot];
    xdot(13)    = 0; % d(mass)/dt
    xdot(14:17) = zeros(4,1);  % d(eta_1:4)/dt = 0 (konstan)

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
% This function now expects the 13-state vector (x_current including mass)
% but the dynamics_fcn internally extracts the mass.
function x_next = rk4_step_quad(x_current, u_applied, dt_val, dynamics_fcn, g_val, l_val, Ixx_val, Iyy_val, Izz_val)
    % Jalankan RK4 hanya untuk 12 state pertama
    assert(isequal(size(x_current), size(dynamics_fcn(x_current, u_applied, g_val, l_val, Ixx_val, Iyy_val, Izz_val))), ...
       'xdot and x_current must have the same dimensions');
    k1 = dynamics_fcn(x_current, u_applied, g_val, l_val, Ixx_val, Iyy_val, Izz_val);
    k2 = dynamics_fcn(x_current + dt_val/2*k1, u_applied, g_val, l_val, Ixx_val, Iyy_val, Izz_val);
    k3 = dynamics_fcn(x_current + dt_val/2*k2, u_applied, g_val, l_val, Ixx_val, Iyy_val, Izz_val);
    k4 = dynamics_fcn(x_current + dt_val*k3, u_applied, g_val, l_val, Ixx_val, Iyy_val, Izz_val);
    
    x_dot = dt_val/6*(k1 + 2*k2 + 2*k3 + k4);
    
    % Perbarui hanya 12 state pertama
    x_next = x_current;
    x_next(1:12) = x_current(1:12) + x_dot(1:12);
    
    % Biarkan mass dan eta tetap (state ke-13 dan 14)
    % x_next(13:14) sudah otomatis tetap karena tidak disentuh
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
    P_pred = P_pred + eye(size(P_pred)) * 1e-6; % <-- Changed from 1e-9 to 1e-7
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


