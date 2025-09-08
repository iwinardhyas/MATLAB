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

% --- Perbaikan di sini: Definisikan SATU vektor parameter utama ---
% Ukuran total parameter = nx (untuk X_initial) + nx (untuk X_ref_param)
n_params = nx + nx; 
all_params_sym = MX.sym('all_params', n_params, 1); 

% Ekstrak parameter individual dari vektor parameter utama
X_initial_param = all_params_sym(1:nx);             % Bagian pertama adalah state awal
X_ref_param_local = all_params_sym(nx+1 : end);     % Bagian kedua adalah referensi

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
    ubw = [ubw; 15*ones(nu,1)];  % Batas atas gaya dorong motor (misal, 10N)
    w0 = [w0; zeros(nu,1)];

    % Variabel State X_{k+1} (hasil dari dinamika dan U_k)
    X_vars{k+2} = MX.sym(['X_' num2str(k+1)], nx, 1); % X_vars{k+2} karena X_vars{1} adalah X_0
    w = {w{:}, X_vars{k+2}};
    lbw = [lbw; -inf*ones(nx,1)];
    ubw = [ubw;  inf*ones(nx,1)];
    w0 = [w0; zeros(nx,1)];

    % Fungsi biaya untuk langkah k
    Q = diag([1000, 1000, 100, 10, 10, 10, 1, 1, 1, 0.1, 0.1, 0.1]); % Bobot error state
    R = diag([0.001, 0.001, 0.1, 0.1]); % Bobot upaya kontrol

    % Menggunakan X_ref_param_local yang diekstrak dari all_params_sym
    J = J + (X_vars{k+1} - X_ref_param_local)'*Q*(X_vars{k+1} - X_ref_param_local) + U_vars{k+1}'*R*U_vars{k+1};
    
    % Kendala dinamika (Multiple Shooting)
    g = {g{:}, F_discrete(X_vars{k+1}, U_vars{k+1}) - X_vars{k+2}};
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