% 1. Setup CasADi
addpath('C:\Users\DELL\Documents\MATLAB\casadi-3.7.0-windows64-matlab2018b');
import casadi.*

% Clear workspace to avoid variable conflicts
clear variables
clear x u
clc
clear all;
close all;

%% 2. Definisikan Parameter Sistem
m = 0.5;    % Massa (kg)
true_grav = 9.81;   % Gravitasi (m/s^2)
l = 0.25;   % Panjang lengan (m)
Ixx = 4.85e-3; % Momen inersia
Iyy = 4.85e-3;
Izz = 8.81e-3;

% Dimensi State dan Input
nx = 12; % [x,y,z,phi,theta,psi,vx_inertial,vy_inertial,vz_inertial,p,q,r]
nu = 4;  % [f1, f2, f3, f4]
nz = 6;       % Measurement dimension

W = [0.5; 0; 0];

M = 2;
% Horizon Prediksi NMPC
N = 30; % Kurangi horizon untuk konvergensi lebih baik
dt = 0.01; % Time step lebih kecil
dt_sub = dt / M;  % pastikan M sudah didefinisikan

nx_ukf = 12; % [px, py, pz, phi, theta, psi, vx, vy, vz, p, q, r]
ny_ukf = 6;  % [px, py, pz, phi, theta, psi] — yang diukur
true_R = 0.05 * eye(ny_ukf);  % atau nilai sebenarnya dari sensor

% Horizon NMPC
N_nmpc = 30; % Langkah waktu prediksi NMPC
dt_nmpc = 0.02; % Ukuran langkah waktu NMPC

% UKF
dt_ukf = 0.01; % Ukuran langkah waktu UKF (lebih cepat dari NMPC untuk estimasi lebih halus)
x0 = zeros(nx,1);
P0 = eye(nx);
% Q0 = 0.01*eye(nx);
% R0 = 0.05*eye(nz);

% Q0: proses
Q0 = diag([0.01 0.01 0.01 ...    % posisi
           0.01 0.01 0.01 ...    % orientasi
           0.001 0.001 0.001 ... % velocity
           0.001 0.001 0.05]);   % angular rate (yaw rate lebih besar)

% R0: pengukuran
R0 = diag([0.4 0.4 0.4 ...       % posisi (lebih besar -> osilasi berkurang)
           0.1 0.1 0.1]);        % orientasi (lebih besar -> osilasi berkurang)


% Simulasi pengukuran
Nsim = 500;

% Waktu Simulasi
T_sim = 3; % Total waktu simulasi (detik)
N_sim_total = T_sim / dt_ukf; % Jumlah langkah simulasi total

% Hovering thrust per motor (pastikan numeric)
thrust_hover_value = double(m * true_grav / 4);  % Force as double
fprintf('Hover thrust per motor: %.3f N\n', thrust_hover_value);

%% --- 2. UKF Implementation ---

%% --- Inisialisasi RAUKF ---
ukf_x_est = zeros(nx,1);    % Estimasi awal
ukf_x_est(3) = 0.1; % Small initial Z position
P = eye(nx);            % Kovariansi state awal
Q = 0.05*eye(nx);       % Kovariansi proses awal
R = 0.05*eye(nz);       % Kovariansi pengukuran awal
chi2_thr = chi2inv(0.95, nz);
lambda0 = 0.2; delta0 = 0.2; a = 5; b = 5;
% Parameter UKF (Unscented Transform)
alpha_ukf = 0.5; % Scaling parameter
kappa_ukf = 0;    % Secondary scaling parameter (sering 0 untuk state tinggi)
beta_ukf = 2;     % Parameter untuk distribusi Gaussian

% Penyimpanan hasil
x_store = zeros(nx,Nsim);
Q_hist = zeros(nx,nx,Nsim);
R_hist = zeros(nz,nz,Nsim);
innovation_hist = zeros(nz,Nsim);

% Fungsi Dinamika UKF (Continuous-time) - Memanggil helper function
% Input: x_ukf (13 states), u_applied (4 motor thrusts)
% Output: xdot_ukf (derivatives for each state)
ukf_dynamics_fcn_handle = @(x, u_applied, g_val, l_val, Ixx_val, Iyy_val, Izz_val) ...
    quadrotor_dynamics_with_wind(x, u_applied, g_val, l_val, Ixx_val, Iyy_val, Izz_val);

% Fungsi Pengukuran UKF (Mengukur posisi dan orientasi)
% Input: x_ukf (13 states)
% Output: y_predicted (6 measurements)
ukf_measurement_fcn = @(x) x(1:6);

% Riwayat (untuk menyimpan hasil)
history_x_ukf = zeros(nx, N_sim_total);
history_y_meas = zeros(ny_ukf, N_sim_total);
history_u_input = zeros(3, N_sim_total); % Menyimpan input kontrol

%% 3. Definisikan Model Dinamika Quadrotor yang Konsisten
% Input simbolik
x = MX.sym('x', nx, 1); % State
u = MX.sym('u', nu, 1); % Input (gaya dorong motor)

% Ekstrak state
px = x(1); py = x(2); pz = x(3);
phi = x(4); theta = x(5); psi = x(6);
vx_inertial = x(7); vy_inertial = x(8); vz_inertial = x(9);
p = x(10); q = x(11); r = x(12);
% === Tambahkan Angin Eksternal ===
% Angin W = [Wx; Wy; Wz] (input tambahan, bukan state)
Wx = W(1); Wy = W(2); Wz = W(3);

% Konversi thrust motor ke total force dan torque
F_total = sum(u);
tau_phi = l * (u(2) - u(4));
tau_theta = l * (u(3) - u(1));
tau_psi = 0.005 * (u(1) - u(2) + u(3) - u(4));

% Rotation matrices
R_b_i = rotz(psi) * roty(theta) * rotx(phi);

% Thrust dalam body frame (mengarah ke atas)
thrust_body = [0; 0; F_total];
% Transform ke inertial frame
thrust_inertial = R_b_i * thrust_body;

% Kecepatan relatif drone terhadap udara
v_rel = [vx_inertial; vy_inertial; vz_inertial] - [Wx; Wy; Wz];

% Drag force (linear model)
C_d = 1; % koefisien drag (bisa disesuaikan)
drag_force = -C_d * v_rel; % bekerja di inertial frame

% Percepatan dalam inertial frame
ax_inertial = (thrust_inertial(1) + drag_force(1)) / m;
ay_inertial = (thrust_inertial(2) + drag_force(2)) / m;
az_inertial = (thrust_inertial(3) + drag_force(3)) / m - true_grav;

% Persamaan Euler untuk angular acceleration
p_dot = (tau_phi + (Iyy - Izz) * q * r) / Ixx;
q_dot = (tau_theta + (Izz - Ixx) * p * r) / Iyy;
r_dot = (tau_psi + (Ixx - Iyy) * p * q) / Izz;

% Kinematic equations untuk orientasi
phi_dot = p + q*sin(phi)*tan(theta) + r*cos(phi)*tan(theta);
theta_dot = q*cos(phi) - r*sin(phi);
psi_dot = q*sin(phi)/cos(theta) + r*cos(phi)/cos(theta);

% State derivative
xdot = [vx_inertial; vy_inertial; vz_inertial; ... % Position rates
        phi_dot; theta_dot; psi_dot; ... % Orientation rates
        ax_inertial; ay_inertial; az_inertial; ... % Velocity rates (inertial)
        p_dot; q_dot; r_dot]; ... % Angular velocity rates

% Fungsi dinamika
W = MX.sym('W', 3, 1);  % wind_x, wind_y, wind_z
f = Function('f', {x, u, W}, {xdot});

% Diskretisasi RK4
X = MX.sym('X', nx, 1);
U = MX.sym('U', nu, 1);
XDOT = f(X, U, W);
k1 = XDOT;
k2 = f(X + dt/2*k1, U, W);
k3 = f(X + dt/2*k2, U, W);
k4 = f(X + dt*k3, U, W);
X_next = X + dt/6*(k1 + 2*k2 + 2*k3 + k4);
F_discrete = Function('F_discrete', {X, U, W}, {X_next});

x_next_sub = X + dt_sub/6 * (k1 + 2*k2 + 2*k3 + k4);
F_sub = Function('F_sub', {X, U, W}, {x_next_sub});

%% 4. Setup NMPC Problem
w = {};
J = 0;
g = {};
lbw = []; ubw = []; w0 = [];
lbg = []; ubg = [];

X_vars = cell(N+1, 1);
U_vars = cell(N, 1);
X_sub_vars = cell(N, M-1);  % State intermediate dalam interval multiple shooting
W_param = MX.sym('W_param', 3*N, 1);  % angin sepanjang horizon

% Parameter vector: [current_state; reference_state]
n_params = nx + nx*(N+1);  % [x0; xref_0; xref_1; ... xref_N]
all_params_sym = MX.sym('all_params', n_params, 1);
X_initial_param = all_params_sym(1:nx);
X_ref_param = all_params_sym(nx+1:end);  % Hasil MX, bukan cell
X_ref_params = cell(N+1, 1);
for k = 0:N
    start_idx = nx + nx*k + 1;
    end_idx = nx + nx*(k+1);
    X_ref_params{k+1} = all_params_sym(start_idx:end_idx);
end
all_params_sym = [all_params_sym; W_param];

% Initial state variable
X_vars{1} = MX.sym('X_0', nx, 1);
w = {w{:}, X_vars{1}};
lbw = [lbw; -inf*ones(nx,1)];
ubw = [ubw; inf*ones(nx,1)];
% w0 = [w0; zeros(nx,1)];

% Initial state constraint
g = {g{:}, X_vars{1} - X_initial_param};
lbg = [lbg; zeros(nx,1)];
ubg = [ubg; zeros(nx,1)];

% --- Generate warm start untuk propagasi awal ---
arg_w0 = [];
x_guess = zeros(nx,1); 

% 1. Initial state
arg_w0 = [arg_w0; x_guess];

% 2. Loop sepanjang horizon
for k = 1:N_nmpc
    % Control guess (hover)
    arg_w0 = [arg_w0; thrust_hover_value*ones(nu,1)];
    
    % Intermediate states (multiple shooting)
    for m = 1:M-1
        arg_w0 = [arg_w0; x_guess];  % bisa pakai interpolasi jika mau
    end
    
    % Next state guess
    arg_w0 = [arg_w0; x_guess];
end


% Main horizon loop
for k = 0:N_nmpc-1
    % Control variables
    U_vars{k+1} = MX.sym(['U_' num2str(k)], nu, 1);
    w = {w{:}, U_vars{k+1}};
    lbw = [lbw; 0.1*thrust_hover_value*ones(nu,1)]; % Min 10% hover thrust
    ubw = [ubw; 3.0*thrust_hover_value*ones(nu,1)]; % Max 300% hover thrust
    arg_w0 = [arg_w0; thrust_hover_value*ones(nu,1)]; % Initialize at hover
    
    % Multiple Shooting Integration
    X_current = X_vars{k+1};
    for m = 1:M-1
        % Intermediate state
        X_sub_vars{k+1, m} = MX.sym(['X_' num2str(k) '_sub_' num2str(m)], nx, 1);
        w = {w{:}, X_sub_vars{k+1, m}};
        lbw = [lbw; -inf*ones(nx,1)];
        ubw = [ubw;  inf*ones(nx,1)];
        
        % Definisi parameter angin sepanjang horizon
        idx = 3*k + 1;                       % indeks angin untuk step ke-k
        W_for_prediction = W_param(idx:idx+2);
        
        % Warm start dari propagasi dinamika
        x_guess = full(F_sub(x_guess, thrust_hover_value*ones(nu,1), zeros(3,1)));
        arg_w0 = [arg_w0; x_guess];
        
        % Constraint: integrasi sub-step
        X_predicted = F_sub(X_current, U_vars{k+1}, W_for_prediction);
        g = {g{:}, X_predicted - X_sub_vars{k+1, m}};
        lbg = [lbg; zeros(nx,1)];
        ubg = [ubg; zeros(nx,1)];
        
        % Update state untuk sub-step berikutnya
        X_current = X_sub_vars{k+1, m};
    end

    
    % Next state variables
    X_vars{k+2} = MX.sym(['X_' num2str(k+1)], nx, 1);
    w = {w{:}, X_vars{k+2}};
    lbw = [lbw; -inf*ones(nx,1)];
    ubw = [ubw; inf*ones(nx,1)];
%     w0 = [w0; zeros(nx,1)];
    x_guess = full(F_discrete(x_guess, thrust_hover_value*ones(nu,1),zeros(3,1)));
    arg_w0 = [arg_w0; x_guess];
    
      % Constraint: integrasi akhir interval
    X_predicted_final = F_sub(X_current, U_vars{k+1},W_for_prediction);
    g = {g{:}, X_predicted_final - X_vars{k+2}};
    lbg = [lbg; zeros(nx,1)];
    ubg = [ubg; zeros(nx,1)];
    
    % Cost function - CONSERVATIVE tuning untuk stabilitas
    % Position tracking dengan weight yang reasonable
    Q = diag([200, 200, 500, ... % px, py, pz
                   80, 80, 40, ...   % phi, theta, psi
                   20, 20, 20, ...      % vx, vy, vz
                   1, 1, 1]);  % p, q, r
    R = diag([1, 1, 1, 1]); % Bobot upaya kontrol
    
    J = J + (X_vars{k+1} - X_ref_params{k+1})' * Q * (X_vars{k+1} - X_ref_params{k+1}) + ...
            U_vars{k+1}' * R * U_vars{k+1};

      
    for m = 1:M-1
        Q_sub = 0.1 * Q;
        alpha = m / M;
        X_ref_interp = (1-alpha) * X_ref_params{k+1} + alpha * X_ref_params{k+2};
        J = J + (X_sub_vars{k+1, m} - X_ref_interp)' * Q_sub * (X_sub_vars{k+1, m} - X_ref_interp);
    end
    
    % Input rate penalty
    if k > 0
        R_rate = 0.1 * eye(nu);
        J = J + (U_vars{k+1} - U_vars{k})' * R_rate * (U_vars{k+1} - U_vars{k});
    end

end

% Terminal cost
Qf = 3 * Q; % Higher terminal weight
J = J + (X_vars{N+1} - X_ref_params{N+1})' * Qf * (X_vars{N+1} - X_ref_params{N+1});


% NLP problem
nlp = struct('f', J, 'x', vertcat(w{:}), 'g', vertcat(g{:}), 'p', all_params_sym);

% Solver options - lebih konservatif untuk stabilitas
solver_options = struct;
solver_options.print_time = false;
solver_options.ipopt.max_iter = 200;
solver_options.ipopt.tol = 1e-4;
solver_options.ipopt.acceptable_tol = 1e-3;
solver_options.ipopt.linear_solver = 'mumps';
solver_options.ipopt.hessian_approximation = 'limited-memory';
solver_options.ipopt.print_level = 0;
solver_options.ipopt.mu_strategy = 'adaptive';

solver = nlpsol('solver', 'ipopt', nlp, solver_options);

%% 5. Simulation Loop
N_sim = T_sim / dt;
history_x = zeros(nx, N_sim + 1);
history_u = zeros(nu, N_sim);
history_x_ref = zeros(nx, N_sim + 1);
history_wind_actual = zeros(3, N_sim); % Menyimpan angin aktual

% Initial state: start closer to first reference point
x_ref_initial = QuadrotorReferenceTrajectory1(0);
current_state = zeros(nx, 1);
current_state(1:3) = x_ref_initial(1:3); % Start at reference position
% current_state(3) = max(current_state(3), 0); % Ensure minimum altitude
history_x(:, 1) = current_state;

% Initialize warm start
% arg_w0 = w0;

fprintf('Starting NMPC simulation...\n');
fprintf('Initial state: [%.3f, %.3f, %.3f]\n', current_state(1:3)');

for i = 1:N_sim
    current_time = (i-1) * dt;
    
    % --- 4.2. UKF: Prediksi & Update ---
    % UKF mendapatkan input kontrol dari NMPC dari langkah sebelumnya.
    % Untuk langkah pertama, asumsikan u_applied_prev = nol
    if i == 1
        u_applied_prev = zeros(nu,1);
    else
%         u_applied_prev = history_u_nmpc(:,i-1);
        u_applied_prev = ones(4,1) * (0.5 * 9.81 / 4);

    end
    
%     actual_wind = zeros(3,1);
    
    if i == 1
        actual_wind = zeros(3,1);
    else
        wind_process_noise = 0.5 * randn(3,1);
        actual_wind = history_wind_actual(:, i-1) + wind_process_noise;
    end
    history_wind_actual(:, i) = actual_wind;
    disp("Actual wind: " + num2str(actual_wind'))
    
    % Buat Pengukuran (z_actual, phi_actual, theta_actual, psi_actual dengan noise)
    measurement_y = ukf_measurement_fcn(current_state) + chol(true_R, 'lower') * randn(ny_ukf, 1);
    
    % Riwayat (untuk menyimpan hasil)
    history_x_ukf = zeros(nx, N_sim_total);
    history_y_meas = zeros(ny_ukf, N_sim_total);
    history_u_input = zeros(3, N_sim_total); % Menyimpan input kontrol

    
     % Prediksi UKF (menggunakan model dinamika quadrotor full)
    [ukf_x_est_pred, ukf_P_pred] = ukf_predict(ukf_x_est, ukf_P, u_applied_prev, ukf_dynamics_fcn_handle, ukf_Q, dt_ukf, true_grav, alpha_ukf, kappa_ukf, beta_ukf, l, Ixx, Iyy, Izz);
    
    % Update UKF
    [ukf_x_est, ukf_P] = ukf_update2(ukf_x_est_pred, ukf_P_pred, measurement_y, ukf_measurement_fcn, ukf_R, alpha_ukf, kappa_ukf, beta_ukf);
    
    % Get reference trajectory
    x_ref_at_current_time = QuadrotorReferenceTrajectory1(current_time);
    
%     W_val = repmat(actual_wind, N, 1);  % misalnya estimasi konstan
    W_val = zeros(3*N,1);
    
    % Build parameter vector
    X_ref_horizon = generate_reference_horizon(current_time, N, dt, @QuadrotorReferenceTrajectory1);
    actual_params = [current_state; reshape(X_ref_horizon, [], 1);W_val];

    
    % Solve NMPC
    try
        tic;
        sol = solver('x0', arg_w0, 'lbx', lbw, 'ubx', ubw, ...
                     'lbg', lbg, 'ubg', ubg, 'p', actual_params);
        solver_time = toc; % catat waktu solver
        fprintf('Step %d: Solver time = %.4f s (dt = %.4f s)\n', i, solver_time, dt);
        solver_times(i) = solver_time;
        
        % Extract optimal control
        opt_w = full(sol.x);
        u_optimal = opt_w(nx + 1 : nx + nu);
        
        % Check solver status (compatible with different CasADi versions)
        try
            if isfield(sol, 'stats') && isfield(sol.stats, 'return_status')
                status = char(sol.stats.return_status);
                if ~strcmp(status, 'Solve_Succeeded')
                    fprintf('Warning: Solver status at step %d: %s\n', i, status);
                end
            end
        catch
            % Ignore status check if not available
        end
        
    catch ME
        fprintf('Solver failed at step %d: %s\n', i, ME.message);
        % Use hover thrust as fallback
        u_optimal = thrust_hover_value * ones(nu, 1);
        opt_w = arg_w0; % Keep previous solution for warm start
    end
    
%     Simulate system forward
    current_state = full(F_discrete(current_state, u_optimal, actual_wind));
    history_x(:, i+1) = current_state;
    current_actual_x = current_state;  % Update untuk pengukuran UKF berikutnya
    history_u(:, i) = u_optimal;
    history_u_nmpc(:, i) = u_optimal;  % Simpan untuk UKF step berikutnya
    history_ukf_x_est(:, i+1) = ukf_x_est;  % Log estimasi UKF
    history_x_ref(:, i) = x_ref_at_current_time;
    
%     Update warm start
    arg_w0 = shift_solution(opt_w, nx, nu, N, M);
    
    % Progress display with error analysis
    if mod(i, 20) == 0
        pos_error = norm(current_state(1:3) - x_ref_at_current_time(1:3));
%         wind_error = norm(estimated_wind - actual_wind);
        fprintf('Step %d/%d, Pos: [%.2f, %.2f, %.2f], Ref: [%.2f, %.2f, %.2f], Error: %.2f\n', ...
                i, N_sim, current_state(1), current_state(2), current_state(3), ...
                x_ref_at_current_time(1), x_ref_at_current_time(2), x_ref_at_current_time(3), pos_error);
        fprintf('           Thrust: [%.2f, %.2f, %.2f, %.2f] N\n', u_optimal');
    end
end

% Final reference point
history_x_ref(:, N_sim + 1) = QuadrotorReferenceTrajectory1(T_sim);

fprintf('Simulation completed!\n');
fprintf('Rata-rata waktu solver: %.4f s, Maksimum: %.4f s\n', ...
        mean(solver_times), max(solver_times));


t_history = (0:N_sim-1) * dt;

% === Ambil data posisi dan referensi ===
x_actual = history_x(1, 1:N_sim);
y_actual = history_x(2, 1:N_sim);
z_actual = history_x(3, 1:N_sim);

x_ref = history_x_ref(1, 1:N_sim);
y_ref = history_x_ref(2, 1:N_sim);
z_ref = history_x_ref(3, 1:N_sim);

% === Plot posisi X, Y, Z ===
figure;
subplot(2,3,1);
plot(t_history, x_actual, 'b', t_history, x_ref, 'r--'); grid on;
xlabel('Time [s]'); ylabel('X Position [m]'); title('X Position');
legend('Actual','Reference');

subplot(2,3,2);
plot(t_history, y_actual, 'b', t_history, y_ref, 'r--'); grid on;
xlabel('Time [s]'); ylabel('Y Position [m]'); title('Y Position');
legend('Actual','Reference');

subplot(2,3,3);
plot(t_history, z_actual, 'b', t_history, z_ref, 'r--'); grid on;
xlabel('Time [s]'); ylabel('Z Position [m]'); title('Z Position');
legend('Actual','Reference');

% === 3D Trajectory - PERBAIKAN INI ===
subplot(2,3,4);
plot3(x_actual, y_actual, z_actual, 'b-', 'LineWidth', 2); 
hold on;
plot3(x_ref, y_ref, z_ref, 'r--', 'LineWidth', 1.5); 
grid on;
xlabel('X [m]'); ylabel('Y [m]'); zlabel('Z [m]');
title('3D Trajectory');
legend('Actual','Reference');

% HAPUS BARIS INI:
% axis equal;

% GANTI DENGAN:
view(45, 30);  % Set viewing angle yang baik
axis tight;    % Fit plot ke data

% ATAU bisa juga gunakan:
% xlim([min([x_actual, x_ref])-0.2, max([x_actual, x_ref])+0.2]);
% ylim([min([y_actual, y_ref])-0.2, max([y_actual, y_ref])+0.2]);
% zlim([min([z_actual, z_ref])-0.1, max([z_actual, z_ref])+0.1]);

% === Motor thrust ===
subplot(2,3,5);
plot(t_history, history_u(:, 1:N_sim)');
xlabel('Time [s]'); ylabel('Thrust [N]');
title('Motor Thrusts'); grid on;
legend('Motor 1','Motor 2','Motor 3','Motor 4');

% === Orientation ===
subplot(2,3,6);
phi = rad2deg(history_x(4, 1:N_sim));
theta = rad2deg(history_x(5, 1:N_sim));
psi = rad2deg(history_x(6, 1:N_sim));
plot(t_history, phi, t_history, theta, t_history, psi);
xlabel('Time [s]'); ylabel('Angle [deg]');
title('Orientation'); grid on;
legend('\phi (roll)','\theta (pitch)','\psi (yaw)');

% === TAMBAHAN: Plot terpisah untuk analisis Z yang lebih detail ===
figure('Name', 'Z Position Analysis');

subplot(2,2,1);
plot(t_history, z_actual, 'b-', 'LineWidth', 2); 
hold on;
plot(t_history, z_ref, 'r--', 'LineWidth', 1.5);
xlabel('Time [s]'); ylabel('Z Position [m]');
title('Z Position Tracking');
legend('Actual','Reference');
grid on;

subplot(2,2,2);
z_error = z_actual - z_ref;
plot(t_history, z_error, 'g-', 'LineWidth', 2);
xlabel('Time [s]'); ylabel('Z Error [m]');
title('Z Position Error');
grid on;

% Proyeksi XZ
subplot(2,2,3);
plot(x_actual, z_actual, 'b-', 'LineWidth', 2); 
hold on;
plot(x_ref, z_ref, 'r--', 'LineWidth', 1.5);
xlabel('X [m]'); ylabel('Z [m]');
title('XZ Projection');
legend('Actual','Reference');
grid on;

% Proyeksi YZ  
subplot(2,2,4);
plot(y_actual, z_actual, 'b-', 'LineWidth', 2); 
hold on;
plot(y_ref, z_ref, 'r--', 'LineWidth', 1.5);
xlabel('Y [m]'); ylabel('Z [m]');
title('YZ Projection');
legend('Actual','Reference');
grid on;

time = (0:N_sim) * dt; % Waktu simulasi

% --- Posisi ---
figure;
subplot(3,1,1);
plot(time, history_x(1,:), 'k-', 'LineWidth', 1.5); hold on;
plot(time, history_ukf_x_est(1,:), 'r--', 'LineWidth', 1.5);
xlabel('Time [s]'); ylabel('x [m]');
legend('True', 'UKF Estimation'); title('UKF Estimation - Position X');

subplot(3,1,2);
plot(time, history_x(2,:), 'k-', 'LineWidth', 1.5); hold on;
plot(time, history_ukf_x_est(2,:), 'r--', 'LineWidth', 1.5);
xlabel('Time [s]'); ylabel('y [m]');
legend('True', 'UKF Estimation'); title('UKF Estimation - Position Y');

subplot(3,1,3);
plot(time, history_x(3,:), 'k-', 'LineWidth', 1.5); hold on;
plot(time, history_ukf_x_est(3,:), 'r--', 'LineWidth', 1.5);
xlabel('Time [s]'); ylabel('z [m]');
legend('True', 'UKF Estimation'); title('UKF Estimation - Position Z');

% --- Orientasi (Roll, Pitch, Yaw) ---
figure;
subplot(3,1,1);
plot(time, history_x(4,:), 'k-', 'LineWidth', 1.5); hold on;
plot(time, history_ukf_x_est(4,:), 'r--', 'LineWidth', 1.5);
xlabel('Time [s]'); ylabel('\phi [rad]');
legend('True', 'UKF Estimation'); title('UKF Estimation - Roll');

subplot(3,1,2);
plot(time, history_x(5,:), 'k-', 'LineWidth', 1.5); hold on;
plot(time, history_ukf_x_est(5,:), 'r--', 'LineWidth', 1.5);
xlabel('Time [s]'); ylabel('\theta [rad]');
legend('True', 'UKF Estimation'); title('UKF Estimation - Pitch');

subplot(3,1,3);
plot(time, history_x(6,:), 'k-', 'LineWidth', 1.5); hold on;
plot(time, history_ukf_x_est(6,:), 'r--', 'LineWidth', 1.5);
xlabel('Time [s]'); ylabel('\psi [rad]');
legend('True', 'UKF Estimation'); title('UKF Estimation - Yaw');

% --- Kecepatan Linier ---
figure;
for k = 1:3
    subplot(3,1,k);
    plot(time, history_x(6+k,:), 'k-', 'LineWidth', 1.5); hold on;
    plot(time, history_ukf_x_est(6+k,:), 'r--', 'LineWidth', 1.5);
    xlabel('Time [s]');
    ylabel(['v', num2str(k), ' [m/s]']);
    legend('True', 'UKF Estimation');
    title(['UKF Estimation - Velocity ', num2str(k)]);
end

% --- Kecepatan Angular ---
figure;
for k = 1:3
    subplot(3,1,k);
    plot(time, history_x(9+k,:), 'k-', 'LineWidth', 1.5); hold on;
    plot(time, history_ukf_x_est(9+k,:), 'r--', 'LineWidth', 1.5);
    xlabel('Time [s]');
    ylabel(['\omega', num2str(k), ' [rad/s]']);
    legend('True', 'UKF Estimation');
    title(['UKF Estimation - Angular Rate ', num2str(k)]);
end


% === Print statistik untuk debugging ===
fprintf('\n=== PLOTTING DEBUG INFO ===\n');
% fprintf('X range: %.3f to %.3f\n', min(x_actual), max(x_actual));
% fprintf('Y range: %.3f to %.3f\n', min(y_actual), max(y_actual));  
% fprintf('Z range: %.3f to %.3f\n', min(z_actual), max(z_actual));
% fprintf('Z reference range: %.3f to %.3f\n', min(z_ref), max(z_ref));
% fprintf('Z error mean: %.6f, std: %.6f\n', mean(z_error), std(z_error));


%% Support Functions
% --- UKF Predict Function (Generic Implementation) ---
% Assuming a generic UKF predict function
function [x_pred, P_pred] = ukf_predict(x_est, P, u, ukf_dynamics_fcn, Q, dt, g_val, alpha, kappa, beta, l_val, Ixx_val, Iyy_val, Izz_val)
    % ===== 1. Setup Parameter Unscented Transform =====
    nx = numel(x_est);
    lambda = alpha^2 * (nx + kappa) - nx;
    Wm = [lambda / (nx + lambda); 0.5 / (nx + lambda) * ones(2*nx, 1)];
    Wc = Wm;
    Wc(1) = Wc(1) + (1 - alpha^2 + beta);

    % ===== 2. Pastikan P Simetris dan Positif-Definit =====
    P = 0.5 * (P + P');  % Simetris
    min_eig = min(eig(P));
    if min_eig <= 0
        P = P + (abs(min_eig) + 1e-3) * eye(nx);  % Regularisasi adaptif
    end
    try
        sqrtP = chol(P, 'lower');
    catch
        warning('P masih tidak positif definit, menggunakan regularisasi tambahan.');
        P = P + 1e-3 * eye(nx);
        sqrtP = chol(P, 'lower');
    end

    % ===== 3. Generate Sigma Points =====
    X_sigma = repmat(x_est(:), 1, 2*nx+1);
    X_sigma(:, 2:nx+1)      = X_sigma(:, 2:nx+1)      + sqrt(nx + lambda) * sqrtP;
    X_sigma(:, nx+2:end)    = X_sigma(:, nx+2:end)    - sqrt(nx + lambda) * sqrtP;

    % ===== 4. Propagasi Sigma Points via RK4 =====
    X_sigma_pred = zeros(nx, 2*nx+1);
    for i = 1:2*nx+1
        xi = X_sigma(:, i);
        k1 = ukf_dynamics_fcn(xi, u, g_val, l_val, Ixx_val, Iyy_val, Izz_val);
        k2 = ukf_dynamics_fcn(xi + dt/2 * k1, u, g_val, l_val, Ixx_val, Iyy_val, Izz_val);
        k3 = ukf_dynamics_fcn(xi + dt/2 * k2, u, g_val, l_val, Ixx_val, Iyy_val, Izz_val);
        k4 = ukf_dynamics_fcn(xi + dt    * k3, u, g_val, l_val, Ixx_val, Iyy_val, Izz_val);
        X_sigma_pred(:, i) = xi + dt/6 * (k1 + 2*k2 + 2*k3 + k4);
    end

    % ===== 5. Hitung Prediksi Mean & Kovariansi =====
    x_pred = X_sigma_pred * Wm;
    P_pred = Q;  % Tambahkan noise proses
    for i = 1:2*nx+1
        diff = X_sigma_pred(:, i) - x_pred;
        P_pred = P_pred + Wc(i) * (diff * diff');
    end

    % ===== 6. Pastikan P_pred Simetris & Positif-Definit =====
    P_pred = 0.5 * (P_pred + P_pred');
    min_eig_pred = min(eig(P_pred));
    if min_eig_pred <= 0
        P_pred = P_pred + (abs(min_eig_pred) + 1e-3) * eye(nx);
    end
    if any(isnan(P_pred(:))) || any(isinf(P_pred(:)))
        error('NaN/Inf terdeteksi di P sebelum Cholesky');
    end
    if any(isnan(X_sigma_pred(:))) || any(isinf(X_sigma_pred(:)))
        error('NaN/Inf di X_sigma_pred');
    end

end


function w_shifted = shift_solution(w_opt, nx, nu, N, M)
    % Versi minimal: otomatis ambil x_last dan u_last dari w_opt
    
    offset = 0;
    X_main = cell(N+1, 1);
    U_all = cell(N, 1);
    X_sub = cell(N, M-1);

    % Ambil X0
    X_main{1} = w_opt(offset+1 : offset+nx);
    offset = offset + nx;

    for k = 1:N
        U_all{k} = w_opt(offset+1 : offset+nu);
        offset = offset + nu;

        for m = 1:M-1
            X_sub{k,m} = w_opt(offset+1 : offset+nx);
            offset = offset + nx;
        end

        X_main{k+1} = w_opt(offset+1 : offset+nx);
        offset = offset + nx;
    end

    % Ambil tebakan terakhir otomatis
    x_last = X_main{end};
    u_last = U_all{end};

    % Bangun warm start
    w_shifted = [];
    w_shifted = [w_shifted; X_main{2}];

    for k = 2:N
        w_shifted = [w_shifted; U_all{k}];
        for m = 1:M-1
            w_shifted = [w_shifted; X_sub{k,m}];
        end
        w_shifted = [w_shifted; X_main{k+1}];
    end

    % Isi step terakhir
    w_shifted = [w_shifted; u_last];
    for m = 1:M-1
        w_shifted = [w_shifted; x_last];
    end
    w_shifted = [w_shifted; x_last];
end

function X_ref_horizon = generate_reference_horizon(t0, N, dt, ref_fun)
    nx = 12;
    X_ref_horizon = zeros(nx, N+1);
    for k = 0:N
        tk = t0 + k*dt;
        X_ref_horizon(:, k+1) = ref_fun(tk);
    end
end


function R_x = rotx(t)
    R_x = [1, 0, 0; 0, cos(t), -sin(t); 0, sin(t), cos(t)];
end

function R_y = roty(t)
    R_y = [cos(t), 0, sin(t); 0, 1, 0; -sin(t), 0, cos(t)];
end

function R_z = rotz(t)
    R_z = [cos(t), -sin(t), 0; sin(t), cos(t), 0; 0, 0, 1];
end