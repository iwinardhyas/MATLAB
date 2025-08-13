% MAIN.M - File Utama untuk Mengintegrasikan Semua Komponen

% 1. Inisialisasi Model State-Space
disp('Membuat model state-space...');
[sys, Ts] = model(); % Fungsi model SS (file model.m)
disp(sys)

% 2. Desain MPC Controller
disp('Membuat MPC controller...');
mpc_ctrl = mpc_controller(sys, Ts); % Fungsi desain MPC (file mpc_controller.m)

% Setpoint atau target
x = [5; 5; 2; 0; 0; 0]; % posisi awal
r = [0; 0; 0; 0; 0; 0]; % Target posisi/output
time = 50;

% 3. Simulasi
disp('Simulasi sistem...');
% sim_data = simulation(sys, mpc_ctrl, time, Ts, x, r); % Fungsi simulasi (file simulation.m)
sim_data = simulation_vertical(sys, mpc_ctrl, time, Ts, x, r); % Fungsi simulasi (file simulation.m)

% 4. Visualisasi
disp('Visualisasi hasil...');
visualization(sim_data,r); % Fungsi visualisasi (file visualization.m)