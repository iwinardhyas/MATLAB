%% Fuzzy Sliding Mode Control (FSMC) untuk Drone Quadrotor

% Bangun FIS untuk setiap kanal kontrol
fis_x     = buildFISFSMC('x');
fis_y     = buildFISFSMC('y');
fis_z     = buildFISFSMC('z');
fis_phi   = buildFISFSMC('phi');
fis_theta = buildFISFSMC('theta');
fis_psi   = buildFISFSMC('psi');

%% Parameter drone
m = 1.5;        % massa (kg)
g = 9.81;       % gravitasi (m/s^2)
Ixx = 0.02; Iyy = 0.02; Izz = 0.04; % momen inersia (kg.m^2)
l = 0.25;       % Panjang lengan (m) - diambil dari main_adaptive_nmpc.m
d_prop_coeff = 0.05; % Koefisien torsi yaw - diambil dari main_adaptive_nmpc.m

dt = 0.01;      % time step
T_total = 10;   % total waktu simulasi (detik)
N = T_total / dt;

%% Inisialisasi state [x y z phi theta psi vx vy vz p q r]'
x = zeros(12, N);
x(:,1) = [0; 0; 0.5; 0; 0; 0; 0; 0; 0; 0; 0; 0];

%% Data untuk plotting
ref_traj = zeros(12, N);
u_history = zeros(4, N); % Sekarang akan menyimpan f1, f2, f3, f4

%% FSMC gains (contoh nilai heuristik)
lambda = [1; 1; 1; 0.5; 0.5; 0.5];
eta    = [0.5; 0.5; 0.5; 0.3; 0.3; 0.3]; % Variabel eta tidak digunakan di fsmc_controller

%% Simulasi kontrol FSMC
for k = 1:N-1
    t = (k-1)*dt;

    if mod(k, 100) == 0
        % Perbaikan: Variabel uz dan tau_phi belum didefinisikan di awal loop
        % Gunakan nilai dari iterasi sebelumnya atau placeholder.
        % Untuk tujuan debugging, ini mungkin lebih baik dicetak setelah perhitungan uz dan tau_phi.
        % Namun, jika Anda ingin mempertahankan format ini, Anda perlu memastikan uz dan tau_phi terdefinisi.
        % Jika tidak, output ini akan menunjukkan nilai dari iterasi sebelumnya.
        if k > 1
             fprintf('t=%.2f | uz=%.2f | tau_phi=%.2f | x_z=%.2f\n', t, u_history(1,k-1) * 4, u_history(2,k-1) * (2*l), x(3,k));
             % u_history sekarang menyimpan f_motor, jadi perlu dikonversi balik untuk display
             % Asumsi f_motor(1)+f_motor(2)+f_motor(3)+f_motor(4) = uz_total
             % Asumsi l*(f_motor(2)-f_motor(4)) = tau_phi
        else
             fprintf('t=%.2f | uz=%.2f | tau_phi=%.2f | x_z=%.2f\n', t, 0, 0, x(3,k)); % Initial values
        end
    end

    % Referensi trajectory
    x_desired = QuadrotorReferenceTrajectory(t, m);
    ref_traj(:,k) = x_desired;

    % Hitung error dan derivatif error
    e     = x_desired(1:6) - x(1:6,k);          % error posisi + rotasi
    e_dot = x_desired(7:12) - x(7:12,k);        % error kecepatan + kecepatan sudut

    % FSMC kontrol (output u_x, u_y, u_z, u_phi, u_theta, u_psi adalah percepatan/percepatan sudut target)
    u_x     = fsmc_controller(x_desired(1), x(1,k), e_dot(1), fis_x, lambda(1));
    u_y     = fsmc_controller(x_desired(2), x(2,k), e_dot(2), fis_y, lambda(2));
    % PERBAIKAN KRITIS: Input ketiga untuk u_z harus e_dot(3) (derivative of error for Z velocity)
    u_z     = fsmc_controller(x_desired(3), x(3,k), e_dot(3), fis_z, lambda(3));
    u_phi   = fsmc_controller(x_desired(4), x(4,k), e_dot(4), fis_phi, lambda(4));
    u_theta = fsmc_controller(x_desired(5), x(5,k), e_dot(5), fis_theta, lambda(5));
    u_psi   = fsmc_controller(x_desired(6), x(6,k), e_dot(6), fis_psi, lambda(6));


    % Hitung thrust total dan torsi target berdasarkan kontrol FSMC
    % Ini adalah GAYA/TORSI KEBUTUHAN dari kontroler
    desired_Fz_total = m * (g + u_z);  % Total thrust yang dibutuhkan untuk sumbu Z
    desired_tau_phi   = Ixx * u_phi;    % Torsi roll yang dibutuhkan
    desired_tau_theta = Iyy * u_theta;  % Torsi pitch yang dibutuhkan
    desired_tau_psi   = Izz * u_psi;    % Torsi yaw yang dibutuhkan
    
    % Batasi thrust total dan torque yang dibutuhkan (opsional, karena nanti f_motor juga dibatasi)
    % desired_Fz_total = max(min(desired_Fz_total, 60), 0); % Contoh batas total thrust
    % desired_tau_phi   = max(min(desired_tau_phi, 2), -2);
    % desired_tau_theta = max(min(desired_tau_theta, 2), -2);
    % desired_tau_psi   = max(min(desired_tau_psi, 1), -1);

    % === PERBAIKAN KRITIS: Konversi Kebutuhan Gaya/Torsi ke Gaya Dorong Motor Individual ===
    % Ini adalah invers dari matriks distribusi gaya/torsi quadrotor
    % Asumsi: F_total = f1+f2+f3+f4; tau_phi = l*(f2-f4); tau_theta = l*(f3-f1); tau_psi = d_prop_coeff*(f1-f2+f3-f4)
    % Dengan f1,f2,f3,f4 adalah gaya dorong motor
    
    f_motor = zeros(4,1);
    f_motor(1) = (desired_Fz_total - desired_tau_theta/l + desired_tau_psi/d_prop_coeff) / 4;
    f_motor(2) = (desired_Fz_total + desired_tau_phi/l + desired_tau_psi/d_prop_coeff) / 4;
    f_motor(3) = (desired_Fz_total + desired_tau_theta/l - desired_tau_psi/d_prop_coeff) / 4;
    f_motor(4) = (desired_Fz_total - desired_tau_phi/l - desired_tau_psi/d_prop_coeff) / 4;
    
    % Batasi gaya dorong motor ke nilai fisik (misalnya, [0, 20] Newton per motor, sesuai NMPC Anda)
    f_motor = max(0, min(f_motor, 20));

    % Sekarang, hitung kembali total thrust dan torsi ACTUAL dari gaya dorong motor yang sudah dibatasi
    % Ini adalah gaya/torsi yang benar-benar diterapkan pada drone
    F_total_actual = sum(f_motor);
    tau_phi_actual = l * (f_motor(2) - f_motor(4));
    tau_theta_actual = l * (f_motor(3) - f_motor(1));
    tau_psi_actual = d_prop_coeff * (f_motor(1) - f_motor(2) + f_motor(3) - f_motor(4));

    % Simpan gaya motor (bukan uz/tau_phi/tau_theta/tau_psi) ke histori jika Anda ingin memplotnya
    u_history(:,k) = f_motor;

    % Update translasi
    R = rotz(x(6,k)) * roty(x(5,k)) * rotx(x(4,k));
    accel = (R * [0; 0; F_total_actual] / m) - [0; 0; g]; % Gunakan F_total_actual
    x(1:3,k+1) = x(1:3,k) + x(7:9,k)*dt;
    x(7:9,k+1) = x(7:9,k) + accel*dt;

    % Update rotasi
    phi = x(4,k); theta = x(5,k);
    p = x(10,k); q = x(11,k); r = x(12,k);
    E = [1 sin(phi)*tan(theta) cos(phi)*tan(theta);
         0 cos(phi) -sin(phi);
         0 sin(phi)/cos(theta) cos(phi)/cos(theta)];
    omega = [p; q; r];
    % PERBAIKAN KRITIS: Gunakan torsi ACTUAL yang dihitung dari f_motor
    ang_vel_dot = [tau_phi_actual/Ixx; tau_theta_actual/Iyy; tau_psi_actual/Izz];
    x(4:6,k+1) = x(4:6,k) + E*omega*dt;
    x(10:12,k+1) = x(10:12,k) + ang_vel_dot*dt;
    
    if x(3,k) < 0
        warning('Drone jatuh pada t = %.2f s', t);
        break;
    end

end

ref_traj(:,N) = QuadrotorReferenceTrajectory(T_total, m);

%% Plot hasil simulasi
t_vec = 0:dt:T_total-dt;

figure;
subplot(3,1,1);
plot(t_vec(1:k), ref_traj(1,1:k), '--', t_vec(1:k), x(1,1:k), 'LineWidth', 1.5);
ylabel('X (m)'); legend('Ref','FSMC');
title('Quadrotor Position Tracking');

subplot(3,1,2);
plot(t_vec(1:k), ref_traj(2,1:k), '--', t_vec(1:k), x(2,1:k), 'LineWidth', 1.5);
ylabel('Y (m)');

subplot(3,1,3);
plot(t_vec(1:k), ref_traj(3,1:k), '--', t_vec(1:k), x(3,1:k), 'LineWidth', 1.5);
ylabel('Z (m)'); xlabel('Time (s)');

% Plot gaya motor (opsional, untuk debugging)
figure;
plot(t_vec(1:k), u_history(:,1:k)');
legend('f1', 'f2', 'f3', 'f4');
title('Individual Motor Forces');
xlabel('Time (s)');
ylabel('Force (N)');