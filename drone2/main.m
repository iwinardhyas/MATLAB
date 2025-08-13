% MAIN.M - File Utama untuk Mengintegrasikan Semua Komponen

% 1. Inisialisasi Model State-Space
disp('Membuat model state-space...');
[sys, Ts] = model(); % Fungsi model SS (file model.m)
disp(sys)

% 2. Desain MPC Controller
disp('Membuat MPC controller...');
mpc_ctrl = mpc_controller(sys, Ts); % Fungsi desain MPC (file mpc_controller.m)

% Setpoint atau target
r = [0; 0; 0; 0; 0; 0]; % Target posisi/output

% 3. Simulasi
disp('Simulasi sistem...');
sim_data = simulation(sys, mpc_ctrl, Ts, r); % Fungsi simulasi (file simulation.m)

% 4. Visualisasi
disp('Visualisasi hasil...');
visualization(sim_data,r); % Fungsi visualisasi (file visualization.m)