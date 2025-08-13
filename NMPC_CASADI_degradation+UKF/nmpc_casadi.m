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
dt = 0.05; % Ukuran langkah waktu diskretisasi (detik)

% Referensi (Target)
x_ref = [0;0;1;0;0;0;0;0;0;0;0;0]; % Contoh: target ketinggian 1m, diam

%% 3. Definisikan Model Dinamika Quadrotor Simbolik (Fungsi State)
% Input simbolik
x = MX.sym('x', nx, 1); % State
u = MX.sym('u', nu, 1); % Input (gaya dorong motor)
% --- Tambahkan simbol untuk massa ---
MASS_f_sym  = MX.sym('mass_f_input', 1);

% Konversi gaya dorong motor ke gaya total dan torsi
% Ini adalah bagian penting yang menghubungkan input motor (u) ke dinamika quadrotor
% Sesuaikan dengan model aktuator quadrotor Anda
F_total = sum(u);
tau_phi = l * (u(2) - u(4));  % Torsi roll
tau_theta = l * (u(3) - u(1)); % Torsi pitch
tau_psi = 0.05 * (u(1) - u(2) + u(3) - u(4)); % Torsi yaw (koefisien d_prop dari dokumentasi)

% Dinamika Quadrotor (seperti di fungsi state Anda sebelumnya)
% Posisi (x, y, z)
px = x(1); py = x(2); pz = x(3);
% Orientasi (phi, theta, psi)
phi = x(4); theta = x(5); psi = x(6);
% Kecepatan Linear (vx, vy, vz)
vx = x(7); vy = x(8); vz = x(9);
% Kecepatan Angular (p, q, r)
p = x(10); q = x(11); r = x(12);

% Rotasi dari Body Frame ke Inertial Frame
R_b_i = rotz(psi) * roty(theta) * rotx(phi);

% Percepatan linear
accel_x = R_b_i(1,3) * F_total / MASS_f_sym   ;
accel_y = R_b_i(2,3) * F_total / MASS_f_sym   ;
accel_z = R_b_i(3,3) * F_total / MASS_f_sym    - g; % Gravitasi ke bawah

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
% f = Function('f', {x, u}, {xdot});
f =  Function('f',{x,u,MASS_f_sym},{xdot},{'x','u','mass'},{'xdot'});

% Diskretisasi Model (Menggunakan metode Runge-Kutta ke-4)
X = MX.sym('X', nx, 1);
U = MX.sym('U', nu, 1);
MASS_F_discrete_sym  = MX.sym('MASS_F_discrete_sym ', 1); % Ini akan menjadi parameter input ke F_discrete
XDOT = f(X, U, MASS_F_discrete_sym );
k1 = XDOT;
k2 = f(X + dt/2*k1, U, MASS_F_discrete_sym );
k3 = f(X + dt/2*k2, U, MASS_F_discrete_sym );
k4 = f(X + dt*k3, U, MASS_F_discrete_sym );
X_next = X + dt/6*(k1 + 2*k2 + 2*k3 + k4);
F_discrete = Function('F_discrete', {X, U, MASS_F_discrete_sym }, {X_next});

%% 4. Rumuskan Masalah Optimasi NMPC (Multiple Shooting)

% Inisialisasi variabel optimasi
w = {}; % Variabel optimasi utama yang akan diumpankan ke solver
J = 0;  % Fungsi biaya
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

nominal_mass_sym = MX.sym('nominal_mass_param_for_nlp', 1); % Simbol ini akan digunakan di kendala

% --- Perbaikan di sini: Definisikan SATU vektor parameter utama ---
% Ukuran total parameter = nx (untuk X_initial) + nx (untuk X_ref_param)
n_params = nx + nx + 1; 
all_params_sym = MX.sym('all_params', n_params, 1); 

% Ekstrak parameter individual dari vektor parameter utama
X_initial_param = all_params_sym(1:nx);             % Bagian pertama adalah state awal
X_ref_param_local = all_params_sym(nx+1 : nx+nx);     % Bagian kedua adalah referensi

nominal_mass_param = all_params_sym(nx+nx+1); % Massa nominal dari parameter solver

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

% Loop untuk sisa horizon prediksi (U_k dan X_{k+1})
for k = 0:N-1
    % Variabel Input U_k
    U_vars{k+1} = MX.sym(['U_' num2str(k)], nu, 1);
    w = {w{:}, U_vars{k+1}};
    lbw = [lbw; 0*ones(nu,1)];   % Batas bawah gaya dorong motor (min 0N)
    ubw = [ubw; 20*ones(nu,1)];  % Batas atas gaya dorong motor (misal, 10N)
    w0 = [w0; zeros(nu,1)];

    % Variabel State X_{k+1} (hasil dari dinamika dan U_k)
    X_vars{k+2} = MX.sym(['X_' num2str(k+1)], nx, 1); % X_vars{k+2} karena X_vars{1} adalah X_0
    w = {w{:}, X_vars{k+2}};
    lbw = [lbw; -inf*ones(nx,1)];
    ubw = [ubw;  inf*ones(nx,1)];
    w0 = [w0; zeros(nx,1)];

    % Fungsi biaya untuk langkah k
    Q = diag([150, 150, 100, 1, 1, 1, 1, 1, 1, 0.1, 0.1, 0.1]); % Bobot error state
    R = diag([0.05, 0.05, 0.1, 0.1]); % Bobot upaya kontrol

    % Menggunakan X_ref_param_local yang diekstrak dari all_params_sym
    J = J + (X_vars{k+1} - X_ref_param_local)'*Q*(X_vars{k+1} - X_ref_param_local) + U_vars{k+1}'*R*U_vars{k+1};
    
    % Kendala dinamika (Multiple Shooting)
    g = {g{:}, F_discrete(X_vars{k+1}, U_vars{k+1},nominal_mass_param) - X_vars{k+2}};
    lbg = [lbg; zeros(nx,1)]; % Kendala kesetaraan (hasil harus 0)
    ubg = [ubg; zeros(nx,1)]; % Kendala kesetaraan (hasil harus 0)
end

% Terminal Cost (Optional): Penalti pada state akhir horizon
% Menggunakan X_ref_param_local
J = J + (X_vars{N+1} - X_ref_param_local)'*Q*(X_vars{N+1} - X_ref_param_local);

% Gabungkan semua variabel, fungsi biaya, dan kendala
% P sekarang adalah all_params_sym tunggal
nlp = struct('f', J, 'x', vertcat(w{:}), 'g', vertcat(g{:}), 'p', all_params_sym); % <--- Perubahan KRITIKAL di sini!

% Konfigurasi solver IPOPT (salah satu yang paling populer)
solver_options = struct;
solver_options.print_time = false;
solver_options.ipopt.max_iter = 100;
solver_options.ipopt.tol = 1e-6;
solver_options.ipopt.linear_solver = 'mumps'; 
solver_options.ipopt.hessian_approximation = 'limited-memory'; 

% Buat solver
solver = nlpsol('solver', 'ipopt', nlp, solver_options);

%% 6. Implementasi Loop NMPC (Simulasi)

% State awal simulasi
current_state = zeros(nx, 1);
current_state(3) = 0; % Misalnya, mulai dari z=0

nominal_mass = 1.0; % kg
mass_at_start = nominal_mass;
payload_mass = 0.5; % Massa payload tambahan (misal 0.5 kg)
time_of_payload_change = 5.0; % Waktu (detik) ketika payload ditambahkan
changed_mass = nominal_mass - payload_mass;

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
    x_ref_at_current_time = QuadrotorReferenceTrajectory3(current_time);
    
    history_x_ref(:, i) = x_ref_at_current_time; % Simpan di kolom 'i'

    % Bangun vektor parameter aktual untuk solver
    % Urutan ini harus cocok dengan all_params_sym: [X_initial_param; X_ref_param_local]
    actual_params = [current_state; x_ref_at_current_time; nominal_mass_value]; % Gunakan referensi yang diperbarui
    % Panggil solver
    sol = solver('x0', arg_w0, 'lbx', lbw, 'ubx', ubw, ...
                 'lbg', lbg, 'ubg', ubg, 'p', actual_params); % Menggunakan actual_params

    % Dapatkan input kontrol optimal pertama dari solusi
    opt_w = full(sol.x);
    % u_optimal = opt_w(nx+1 : nx+nu); % Ini bergantung pada urutan w. Ambil dari U_vars{1}
    
    % Dapatkan input U_0 dari solusi optimal
    % Karena w = [X_0, U_0, X_1, U_1, ..., X_N], U_0 adalah nx+1 sampai nx+nu
    u_optimal = opt_w(nx + 1 : nx + nu); 

    history_u(:, i) = u_optimal;

    % Prediksi state berikutnya menggunakan model diskrit
    % current_state = full(F_discrete(current_state, u_optimal));
    % history_x(:, i+1) = current_state;

        % Prediksi state berikutnya menggunakan model diskrit
    current_state = full(F_discrete(current_state, u_optimal,current_sim_mass));
    history_x(:, i+1) = current_state;
    % Warm start untuk iterasi berikutnya (geser tebakan awal)
    arg_w0 = shift_solution(opt_w, nx, nu, N);
    
    % Tampilkan progres (opsional)
    if mod(i, 10) == 0
        fprintf('Iterasi %d dari %d, Z: %.2f m\n', i, N_sim, current_state(3));
    end
end

disp('Simulasi selesai.');
history_x_ref(:, N_sim + 1) = QuadrotorReferenceTrajectory(T_sim);

PlotTrajectory;
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