% 1. Setup CasADi
addpath('C:\Users\DELL\Documents\MATLAB\casadi-3.7.0-windows64-matlab2018b'); % Ganti dengan path instalasi CasADi Anda
import casadi.*

%% 2. Definisikan Parameter Sistem
% Sama seperti yang Anda miliki di model quadrotor Anda
nominal_mass_value  = 1;    % Massa (kg)
g = 9.81;   % Gravitasi (m/s^2)
l = 0.25;   % Panjang lengan (m)
Ixx = 4.85e-3; % Momen inersia
Iyy = 4.85e-3;
Izz = 8.81e-3;

% Dimensi State dan Input
nx = 12; % Posisi (x,y,z), Orientasi (phi,theta,psi), Kecepatan Linear (vx,vy,vz), Kecepatan Angular (p,q,r)
nu = 4;  % Dorong masing-masing motor (f1, f2, f3, f4)

% Horizon Prediksi NMPC
N = 20; % Prediction horizon (langkah waktu)
dt = 0.05; % Ukuran langkah waktu diskretisasi (detik) (Ini akan menjadi parameter CasADi)

% Referensi (Target) - Hanya sebagai placeholder di sini, nilai aktual dari luar
x_ref = [0;0;1;0;0;0;0;0;0;0;0;0];

%% 3. Definisikan Model Dinamika Quadrotor Simbolik (Fungsi State)
% Input simbolik
x = MX.sym('x', nx, 1); % State
u = MX.sym('u', nu, 1); % Input (gaya dorong motor)
MASS_f_sym  = MX.sym('mass_f_input', 1); % Massa, bisa nominal atau diestimasi

% Konversi gaya dorong motor ke gaya total dan torsi
F_total = sum(u);
tau_phi = l * (u(2) - u(4));  % Torsi roll
tau_theta = l * (u(3) - u(1)); % Torsi pitch
tau_psi = 0.05 * (u(1) - u(2) + u(3) - u(4)); % Torsi yaw (koefisien d_prop dari dokumentasi)

% Dinamika Quadrotor (seperti di fungsi state Anda sebelumnya)
px = x(1); py = x(2); pz = x(3);
phi = x(4); theta = x(5); psi = x(6);
vx = x(7); vy = x(8); vz = x(9);
p = x(10); q = x(11); r = x(12);

% Rotasi dari Body Frame ke Inertial Frame
R_b_i = rotz(psi) * roty(theta) * rotx(phi);

% Percepatan linear
accel_x = R_b_i(1,3) * F_total / MASS_f_sym;
accel_y = R_b_i(2,3) * F_total / MASS_f_sym;
accel_z = R_b_i(3,3) * F_total / MASS_f_sym - g; % Gravitasi ke bawah

% Percepatan angular (persamaan Euler)
p_dot = (tau_phi + (Iyy - Izz) * q * r) / Ixx;
q_dot = (tau_theta + (Izz - Ixx) * p * r) / Iyy;
r_dot = (tau_psi + (Ixx - Iyy) * p * q) / Izz;

% State derivative (dx/dt)
xdot = [vx; vy; vz; ... % Posisi
        p + q*sin(phi)*tan(theta) + r*cos(phi)*tan(theta); ... % phi_dot
        q*cos(phi) - r*sin(phi); ... % theta_dot
        q*sin(phi)/cos(theta) + r*cos(phi)/cos(theta); ... % psi_dot
        accel_x; accel_y; accel_z; ... % Kecepatan Linear
        p_dot; q_dot; r_dot]; % Kecepatan Angular

% Fungsi CasADi untuk dinamika sistem (Continuous-time)
f = Function('f',{x,u,MASS_f_sym},{xdot},{'x','u','mass'},{'xdot'});

% Diskretisasi Model (Menggunakan metode Runge-Kutta ke-4)
X = MX.sym('X', nx, 1);
U = MX.sym('U', nu, 1);
MASS_F_discrete_sym = MX.sym('MASS_F_discrete_sym', 1); % Ini akan menjadi parameter input ke F_discrete
DT_sym_discrete = MX.sym('DT_sym_discrete', 1); % dt sebagai parameter simbolik

XDOT_rk = f(X, U, MASS_F_discrete_sym);
k1 = XDOT_rk;
k2 = f(X + DT_sym_discrete/2*k1, U, MASS_F_discrete_sym);
k3 = f(X + DT_sym_discrete/2*k2, U, MASS_F_discrete_sym);
k4 = f(X + DT_sym_discrete*k3, U, MASS_F_discrete_sym);
X_next = X + DT_sym_discrete/6*(k1 + 2*k2 + 2*k3 + k4);
F_discrete = Function('F_discrete', {X, U, MASS_F_discrete_sym, DT_sym_discrete}, {X_next});

%% 4. Rumuskan Masalah Optimasi NMPC (Multiple Shooting)

% Inisialisasi variabel optimasi
w = {}; % Variabel optimasi utama yang akan diumpankan ke solver
J = MX.zeros(1,1);  % <<<<<<< PERUBAHAN PENTING DI SINI: Inisialisasi J sebagai skalar MX eksplisit
g = {}; % Kendala (kesetaraan dan ketidaksetaraan)

% Batasan untuk variabel optimasi
lbw = []; % Lower bounds untuk variabel optimasi
ubw = []; % Upper bounds untuk variabel optimasi
w0 = [];  % Tebakan awal variabel optimasi

lbg = []; % Lower bounds untuk kendala
ubg = []; % Upper bounds untuk kendala

% Inisialisasi daftar untuk menyimpan referensi variabel state dan input
X_vars = cell(N+1, 1); % X_0, X_1, ..., X_N
U_vars = cell(N, 1);   % U_0, U_1, ..., U_{N-1}

% --- Definisikan semua parameter eksternal untuk solver CasADi ---
% Urutan parameter ini harus konsisten saat memanggil solver
% Parameter 1: State awal (nx)
% Parameter 2: Referensi lintasan (nx)
% Parameter 3: Massa nominal/estimasi (1)
% Parameter 4-6: Parameter model daya (3) - P0t, kT, kV
% Parameter 7: Bobot energi (1)
% Parameter 8: Langkah waktu NMPC (1)

num_power_params = 3; % P0t, kT, kV
% HITUNG ULANG TOTAL PARAMETER SECARA EKSPLISIT
n_total_params = nx + nx + 1 + num_power_params + 1 + 1; % Total: 12 + 12 + 1 + 3 + 1 + 1 = 30
all_params_sym = MX.sym('all_params', n_total_params, 1);

% --- Ekstraksi Parameter Individual dari vektor all_params_sym ---
idx = 0; % Reset indeks

X_initial_param = all_params_sym(idx + 1 : idx + nx);
idx = idx + nx;

X_ref_param_local = all_params_sym(idx + 1 : idx + nx);
idx = idx + nx;

nominal_mass_param = all_params_sym(idx + 1 : idx + 1);
idx = idx + 1;

% Parameter model daya (BARU)
P0t_sym = all_params_sym(idx + 1 : idx + 1);
idx = idx + 1;
kT_sym  = all_params_sym(idx + 1 : idx + 1);
idx = idx + 1;
kV_sym  = all_params_sym(idx + 1 : idx + 1);
idx = idx + 1;
estimated_power_params_sym = struct('P0t', P0t_sym, 'kT', kT_sym, 'kV', kV_sym);

% Bobot dan dt (BARU)
w_energy_sym = all_params_sym(idx + 1 : idx + 1);
idx = idx + 1;
dt_nmpc_sym = all_params_sym(idx + 1 : idx + 1);
idx = idx + 1;

% Pastikan variabel 'idx' berakhir dengan nilai n_total_params.
fprintf('Debug: Final idx = %d, n_total_params = %d\n', idx, n_total_params);


% --- Definisikan Variabel Optimasi untuk Setiap Langkah Horizon ---
% Langkah 0: Variabel State X_0
X_vars{1} = MX.sym('X_0', nx, 1);
w = {w{:}, X_vars{1}};
lbw = [lbw; -inf*ones(nx,1)]; % Umumnya state tidak dibatasi keras
ubw = [ubw;  inf*ones(nx,1)];
w0 = [w0; zeros(nx,1)];

% Kendala: State awal dari horizon prediksi harus sama dengan parameter state awal
g = {g{:}, X_vars{1} - X_initial_param}; % Menggunakan X_initial_param dari all_params_sym
lbg = [lbg; zeros(nx,1)];
ubg = [ubg; zeros(nx,1)];

% Parameter Fisika untuk Model Daya (jika dibutuhkan, bisa didefinisikan di sini atau hardcoded di power_model)
% Untuk contoh sederhana, kita tidak perlu membuatnya simbolik dan meneruskannya.
physics_params_sym = struct('g', g); % Jika quadcopter_power_model memerlukannya

% Loop untuk sisa horizon prediksi (U_k dan X_{k+1})
for k = 0:N-1
    % Variabel Input U_k
    U_vars{k+1} = MX.sym(['U_' num2str(k)], nu, 1);
    w = {w{:}, U_vars{k+1}};
    lbw = [lbw; 0*ones(nu,1)];    % Batas bawah gaya dorong motor (min 0N)
    ubw = [ubw; 20*ones(nu,1)];   % Batas atas gaya dorong motor (misal, 20N per motor)
    w0 = [w0; zeros(nu,1)];

    % Variabel State X_{k+1} (hasil dari dinamika dan U_k)
    X_vars{k+2} = MX.sym(['X_' num2str(k+1)], nx, 1); % X_vars{k+2} karena X_vars{1} adalah X_0
    w = {w{:}, X_vars{k+2}};
    lbw = [lbw; -inf*ones(nx,1)];
    ubw = [ubw;  inf*ones(nx,1)];
    w0 = [w0; zeros(nx,1)];

    % --- Fungsi biaya untuk langkah k ---
    % Bobot error state (Q) dan upaya kontrol (R)
    Q = diag([150, 150, 250, 1, 1, 1, 1, 1, 1, 0.1, 0.1, 0.1]); % Bobot error state
    R = diag([0.03, 0.03, 0.03, 0.03]); % Bobot upaya kontrol (untuk masing-masing motor)

    % Komponen Biaya Pelacakan
    tracking_error = X_vars{k+1} - X_ref_param_local;
    term_tracking = (tracking_error)'*Q*(tracking_error); % Simpan di variabel sementara
    
    J = J + term_tracking;
    
%     DEBUG: Cek J setelah penambahan tracking term (hanya di iterasi pertama)
    if k == 0
        fprintf('Debug: Dimensi J setelah tracking term (k=0): [%d, %d]\n', J.size());
    end

    % Simpan ke parameter aktual
    actual_params = [current_state;
                     x_ref_at_current_time;
                     nominal_mass_value;
                     P0t_sym;
                     kT_sym;
                     kV_sym;
                     w_energy_sym;
                     dt];
    
    % Komponen Biaya Upaya Kontrol
    term_control = U_vars{k+1}'*R*U_vars{k+1};
    J = J + term_control;
    
    % Komponen Biaya Optimasi Energi (BARU)
    predicted_power_k = quadcopter_power_model(X_vars{k+1}, ...
                                                sum(U_vars{k+1}), ...
                                               estimated_power_params_sym, ...
                                               physics_params_sym);
    term_energy = (w_energy_sym * predicted_power_k * dt_nmpc_sym);
    J = J + term_energy;
end

%% 5. Setup Solver NLP
% Concatenate semua variabel optimasi menjadi satu vektor
w_all = vertcat(w{:});

% Concatenate semua kendala menjadi satu vektor
g_all = vertcat(g{:});

% Setup masalah NLP
nlp_problem = struct('f', J, 'x', w_all, 'g', g_all, 'p', all_params_sym); % 'p' adalah parameter eksternal

% Opsi Solver (IPOPT)
solver_opts = struct;
solver_opts.ipopt.max_iter = 200;
solver_opts.ipopt.print_level = 0; % 0 untuk tidak menampilkan output, 3 untuk verbose
solver_opts.print_time = false; % Jangan tampilkan waktu solver CasADi
solver_opts.ipopt.acceptable_tol = 1e-6;
solver_opts.ipopt.acceptable_obj_change_tol = 1e-6;
solver_opts.ipopt.linear_solver = 'mumps'; % Atau 'ma57', 'ma27' jika mumps tidak ada/lambat

% Buat solver CasADi
solver = nlpsol('solver', 'ipopt', nlp_problem, solver_opts);

% fprintf('Iterasi %d: Status solver = %s, fval = %.4f\n', ...
%     i, solver.stats.return_status, full(sol.f));


% --- Fungsi untuk mengekstrak solusi dari w_all (ini sudah ada dan benar) ---
evaluate_solution = Function('evaluate_solution', {w_all}, {vertcat(X_vars{:}), vertcat(U_vars{:})}, {'w_opt'}, {'X_extracted', 'U_extracted'});


% --- Definisikan Fungsi untuk Membuat Tebakan Awal yang Digeser (Shifted Initial Guess) ---
% Input ke fungsi ini adalah w_opt dari solusi sebelumnya
w_opt_input_sym = MX.sym('w_opt_input', w_all.size()); % Buat variabel simbolik untuk input fungsi ini

% --- Ekstrak variabel state dan input secara simbolik dari w_opt_input_sym ---
% Kita harus menduplikasi logika ekstraksi dari w_all, tetapi dengan w_opt_input_sym
extracted_x_sym = cell(N+1, 1);
extracted_u_sym = cell(N, 1);
offset_sym = 0;

% Ekstrak X_0_sym (state awal dari w_opt_input_sym)
extracted_x_sym{1} = w_opt_input_sym(offset_sym + 1 : offset_sym + nx);
offset_sym = offset_sym + nx;

% Ekstrak U_0_sym, X_1_sym, U_1_sym, X_2_sym, ..., U_{N-1}_sym, X_N_sym
for k = 0:N-1
    extracted_u_sym{k+1} = w_opt_input_sym(offset_sym + 1 : offset_sym + nu);
    offset_sym = offset_sym + nu;
    
    extracted_x_sym{k+2} = w_opt_input_sym(offset_sym + 1 : offset_sym + nx);
    offset_sym = offset_sym + nx;
end

% --- Bangun tebakan awal yang digeser secara simbolik ---
% X_0 baru adalah X_1 dari solusi sebelumnya
w_shifted_output_sym = extracted_x_sym{2}; % X_1 lama menjadi X_0 baru

% U_k baru adalah U_{k+1} dari solusi sebelumnya (geser ke kiri)
% X_{k+1} baru adalah X_{k+2} dari solusi sebelumnya (geser ke kiri)
for k = 0:N-2 % Iterasi dari U_0 hingga U_{N-2}
    w_shifted_output_sym = vertcat(w_shifted_output_sym, extracted_u_sym{k+2}); % U_{k+1} lama menjadi U_k baru
    w_shifted_output_sym = vertcat(w_shifted_output_sym, extracted_x_sym{k+3}); % X_{k+2} lama menjadi X_{k+1} baru
end

% Tambahkan tebakan awal untuk input dan state terakhir dari horizon
% Ini sering diset ke nilai default (misal, nol) atau diulang dari nilai terakhir yang tersedia
% Pastikan dimensinya cocok dengan sisa w_all
w_shifted_output_sym = vertcat(w_shifted_output_sym, zeros(nu, 1)); % U_{N-1} (terakhir) bisa diset nol
w_shifted_output_sym = vertcat(w_shifted_output_sym, zeros(nx, 1)); % X_N (terakhir) bisa diset nol


% Definisikan fungsi CasADi untuk generator tebakan yang digeser
shifted_guess_generator = Function('shifted_guess_generator', ...
                                   {w_opt_input_sym}, ...
                                   {w_shifted_output_sym}, ...
                                   {'w_opt'}, ...
                                   {'w_shifted'});

% ... (lanjutan kode untuk definisi solver dan sisa file)
%% 6. Implementasi Loop NMPC (Simulasi)

% State awal simulasi
current_state = zeros(nx, 1);
current_state(3) = 0; % Misalnya, mulai dari z=0

nominal_mass = 1.0; % kg
mass_at_start = nominal_mass;
payload_mass = 0.5; % Massa payload tambahan (misal 0.5 kg)
time_of_payload_change = 6.0; % Waktu (detik) ketika payload ditambahkan
changed_mass = nominal_mass + payload_mass;

current_sim_mass = mass_at_start;

% Loop simulasi
T_sim = 8; % Total waktu simulasi
N_sim = T_sim / dt; % Jumlah langkah simulasi
history_x = zeros(nx, N_sim + 1);
history_u = zeros(nu, N_sim);
history_x(:, 1) = current_state;

% Warm start (gunakan solusi sebelumnya sebagai tebakan awal)
arg_w0 = w0; % Inisialisasi tebakan awal
history_x_ref = zeros(nx, N_sim + 1); % nx = 12, N_sim+1 karena menyertakan titik awal

disp('Memulai Simulasi NMPC...');
for i = 1:N_sim
    % --- Perubahan di sini: Bangun vektor parameter aktual untuk solver ---
    % actual_params = [current_state; x_ref]; % Gabungkan nilai aktual X_initial dan X_ref
    % Hitung waktu saat ini
    current_time = (i-1) * dt; % Waktu aktual pada iterasi ini

    if current_time >= time_of_payload_change && current_sim_mass == mass_at_start
        current_sim_mass = changed_mass;
        fprintf('--- Perubahan Payload! Massa drone sekarang: %.2f kg pada t=%.2f s ---\n', current_sim_mass, current_time);
    end
    % --- PENTING: Panggil fungsi trajectory di sini ---
    % Dapatkan referensi trajectory untuk waktu saat ini
    x_ref_at_current_time = QuadrotorReferenceTrajectory4(current_time);
    
    history_x_ref(:, i) = x_ref_at_current_time; % Simpan di kolom 'i'

    % Bangun vektor parameter aktual untuk solver
    % Urutan ini harus cocok dengan all_params_sym: [X_initial_param; X_ref_param_local]
    actual_params = [current_state; x_ref_at_current_time; nominal_mass_value;P0t_sym;
                 kT_sym;
                 kV_sym;
                 w_energy_sym;
                 dt]; % Gunakan referensi yang diperbarui
    % Panggil solver
    sol = solver('x0', arg_w0, 'lbx', lbw, 'ubx', ubw, ...
                 'lbg', lbg, 'ubg', ubg, 'p', actual_params); % Menggunakan actual_params

    % Dapatkan input kontrol optimal pertama dari solusi
    opt_w = full(sol.x);
    % u_optimal = opt_w(nx+1 : nx+nu); % Ini bergantung pada urutan w. Ambil dari U_vars{1}
    
    % Dapatkan input U_0 dari solusi optimal
    % Karena w = [X_0, U_0, X_1, U_1, ..., X_N], U_0 adalah nx+1 sampai nx+nu
%     u_optimal = opt_w(nx + 1 : nx + nu); 
    u_optimal = opt_w(13:16);
    
    if isa(u_optimal, 'casadi.MX')
        u_optimal = full(u_optimal);
    end
    disp(size(opt_w));             % Harusnya sekitar 320x1 jika N=20
    disp(size(u_optimal));         % Harusnya 4x1
    disp(class(u_optimal));        % Harusnya 'double'

    u_optimal = full(opt_w(nx + 1 : nx + nu));


    % Prediksi state berikutnya menggunakan model diskrit
    % current_state = full(F_discrete(current_state, u_optimal));
    % history_x(:, i+1) = current_state;

        % Prediksi state berikutnya menggunakan model diskrit
    current_state = full(F_discrete(current_state, u_optimal, current_sim_mass, dt));
    if iscell(current_state)
        current_state = current_state{1}; % Ambil elemen pertama jika output berupa cell
    end
    current_state = full(current_state); % Konversi ke double

    % Warm start untuk iterasi berikutnya (geser tebakan awal)
    arg_w0 = shift_solution(opt_w, nx, nu, N);
    
%     % Tampilkan progres (opsional)
%     if mod(i, 10) == 0
%         fprintf('Iterasi %d dari %d, Z: %.2f m\n', i, N_sim, current_state(3));
%     end
end

disp('Simulasi selesai.');
history_x_ref(:, N_sim + 1) = QuadrotorReferenceTrajectory4(T_sim);

% PlotTrajectory;

%% --- 5. Plot Hasil ---
time_vec = 0:dt:T_sim;

figure('Name', 'Quadrotor Position Tracking');
subplot(3,1,1);
plot(time_vec, history_x(1,:), 'b', 'LineWidth', 1.5); hold on;
% plot(time_vec, history_ukf_x_est(1,:), 'g--', 'LineWidth', 1.0);
plot(time_vec, history_x_ref(1,:), 'r:', 'LineWidth', 1.0);
title('Quadrotor X Position');
xlabel('Time (s)'); ylabel('Position X (m)');
legend('Actual X', 'Reference X');
grid on;

subplot(3,1,2);
plot(time_vec, history_x(2,:), 'b', 'LineWidth', 1.5); hold on;
% plot(time_vec, history_ukf_x_est(2,:), 'g--', 'LineWidth', 1.0);
plot(time_vec, history_x_ref(2,:), 'r:', 'LineWidth', 1.0);
title('Quadrotor Y Position');
xlabel('Time (s)'); ylabel('Position Y (m)');
legend('Actual Y', 'Reference Y');
grid on;

subplot(3,1,3);
plot(time_vec, history_x(3,:), 'b', 'LineWidth', 1.5); hold on;
% plot(time_vec, history_ukf_x_est(3,:), 'g--', 'LineWidth', 1.0);
plot(time_vec, history_x_ref(3,:), 'r:', 'LineWidth', 1.0);
title('Quadrotor Z Position');
xlabel('Time (s)'); ylabel('Position Z (m)');
legend('Actual Z', 'Reference Z');
grid on;

figure('Name', 'Quadrotor Velocity Tracking');
subplot(3,1,1);
plot(time_vec, history_x(7,:), 'b', 'LineWidth', 1.5); hold on;
% plot(time_vec, history_ukf_x_est(7,:), 'g--', 'LineWidth', 1.0);
plot(time_vec, history_x_ref(7,:), 'r:', 'LineWidth', 1.0);
title('Quadrotor X Velocity');
xlabel('Time (s)'); ylabel('Velocity X (m/s)');
legend('Actual VX', 'Reference VX');
grid on;

subplot(3,1,2);
plot(time_vec, history_x(8,:), 'b', 'LineWidth', 1.5); hold on;
% plot(time_vec, history_ukf_x_est(8,:), 'g--', 'LineWidth', 1.0);
plot(time_vec, history_x_ref(8,:), 'r:', 'LineWidth', 1.0);
title('Quadrotor Y Velocity');
xlabel('Time (s)'); ylabel('Velocity Y (m/s)');
legend('Actual VY', 'Reference VY');
grid on;

subplot(3,1,3);
plot(time_vec, history_x(9,:), 'b', 'LineWidth', 1.5); hold on;
% plot(time_vec, history_ukf_x_est(9,:), 'g--', 'LineWidth', 1.0);
plot(time_vec, history_x_ref(9,:), 'r:', 'LineWidth', 1.0);
title('Quadrotor Z Velocity');
xlabel('Time (s)'); ylabel('Velocity Z (m/s)');
legend('Actual VZ', 'Reference VZ');
grid on;

figure('Name', 'Quadrotor Orientation Tracking');
subplot(3,1,1);
plot(time_vec, rad2deg(history_x(4,:)), 'b', 'LineWidth', 1.5); hold on;
% plot(time_vec, rad2deg(history_ukf_x_est(4,:)), 'g--', 'LineWidth', 1.0);
plot(time_vec, rad2deg(history_x_ref(4,:)), 'r:', 'LineWidth', 1.0);
title('Quadrotor Roll (Phi)');
xlabel('Time (s)'); ylabel('Roll (deg)');
legend('Actual Roll', 'Reference Roll');
grid on;

subplot(3,1,2);
plot(time_vec, rad2deg(history_x(5,:)), 'b', 'LineWidth', 1.5); hold on;
% plot(time_vec, rad2deg(history_ukf_x_est(5,:)), 'g--', 'LineWidth', 1.0);
plot(time_vec, rad2deg(history_x_ref(5,:)), 'r:', 'LineWidth', 1.0);
title('Quadrotor Pitch (Theta)');
xlabel('Time (s)'); ylabel('Pitch (deg)');
legend('Actual Pitch', 'Reference Pitch');
grid on;

subplot(3,1,3);
plot(time_vec, rad2deg(history_x(6,:)), 'b', 'LineWidth', 1.5); hold on;
% plot(time_vec, rad2deg(history_ukf_x_est(6,:)), 'g--', 'LineWidth', 1.0);
plot(time_vec, rad2deg(history_x_ref(6,:)), 'r:', 'LineWidth', 1.0);
title('Quadrotor Yaw (Psi)');
xlabel('Time (s)'); ylabel('Yaw (deg)');
legend('Actual Yaw', 'Reference Yaw');
grid on;


% figure('Name', 'Mass Estimation and Control Inputs');
% subplot(2,1,1);
% plot(time_vec, history_actual_mass, 'b', 'LineWidth', 1.5); hold on;
% plot(time_vec, history_ukf_x_est(end,:), 'r--', 'LineWidth', 1.0);
% title('Mass Actual vs. UKF Estimated');
% xlabel('Time (s)'); ylabel('Mass (kg)');
% legend('Actual Mass', 'UKF Estimated Mass');
% grid on;

% subplot(2,1,2);
% plot(time_vec(1:end-1), history_u_nmpc(1,:), 'r', 'LineWidth', 1.0); hold on;
% plot(time_vec(1:end-1), history_u_nmpc(2,:), 'g', 'LineWidth', 1.0);
% plot(time_vec(1:end-1), history_u_nmpc(3,:), 'b', 'LineWidth', 1.0);
% plot(time_vec(1:end-1), history_u_nmpc(4,:), 'k', 'LineWidth', 1.0);
% title('NMPC Control Inputs (Motor Thrusts)');
% xlabel('Time (s)'); ylabel('Thrust (N)');
% legend('Motor 1', 'Motor 2', 'Motor 3', 'Motor 4');
% grid on;

%% 7. Fungsi Pembantu untuk Warm Start
function w_shifted = shift_solution(w_opt, nx, nu, N)
    % Fungsi untuk menggeser solusi NMPC untuk warm start
    % Asumsi w_opt = [X_0, U_0, X_1, U_1, ..., U_{N-1}, X_N]
    % Dimana X_k adalah state di awal interval ke-k, dan U_k adalah input di interval ke-k.

    % Ekstrak state dan input dari w_opt
    extracted_x = cell(N+1, 1);
    extracted_u = cell(N, 1);

    offset = 0;
    % Extract X_0
    extracted_x{1} = w_opt(offset + 1 : offset + nx);
    offset = offset + nx;

    % Extract U_0, X_1, U_1, X_2, ..., U_{N-1}, X_N
    for k = 0:N-1
        extracted_u{k+1} = w_opt(offset + 1 : offset + nu);
        offset = offset + nu;
        
        extracted_x{k+2} = w_opt(offset + 1 : offset + nx);
        offset = offset + nx;
    end
    
    % Bangun tebakan awal yang digeser
    w_shifted = [];
    
    % X_0 baru adalah X_1 dari solusi sebelumnya
    w_shifted = [w_shifted; extracted_x{2}]; % X_1 lama menjadi X_0 baru

    % U_k baru adalah U_{k+1} dari solusi sebelumnya (geser ke kiri)
    % X_{k+1} baru adalah X_{k+2} dari solusi sebelumnya (geser ke kiri)
    for k = 0:N-2 % Iterasi dari U_0 hingga U_{N-2}
        w_shifted = [w_shifted; extracted_u{k+2}]; % U_{k+1} lama menjadi U_k baru
        w_shifted = [w_shifted; extracted_x{k+3}]; % X_{k+2} lama menjadi X_{k+1} baru
    end
    
    % Untuk U_{N-1} terakhir, kita ulang U_{N-1} lama
    w_shifted = [w_shifted; extracted_u{N}];
    
    % Untuk X_N terakhir, kita ulang X_N lama
    w_shifted = [w_shifted; extracted_x{N+1}];

end

% Fungsi rotasi (untuk model dinamika)
function R_x = rotx(t)
    R_x = [1, 0, 0; 0, cos(t), -sin(t); 0, sin(t), cos(t)];
end
function R_y = roty(t)
    R_y = [cos(t), 0, sin(t); 0, 1, 0; -sin(t), 0, cos(t)];
end
function R_z = rotz(t)
    R_z = [cos(t), -sin(t), 0; sin(t), cos(t), 0; 0, 0, 1];
end