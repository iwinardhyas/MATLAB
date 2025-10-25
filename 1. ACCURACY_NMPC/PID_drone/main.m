addpath('C:\Users\DELL\Documents\MATLAB\casadi-3.7.0-windows64-matlab2018b');
import casadi.*

% Clear workspace to avoid variable conflicts
clear variables
clear x u
close all;
clc


N = 10; % Kurangi horizon untuk konvergensi lebih baik
dt = 0.1; % Time step lebih kecil
M = 1;
T_sim = 50;
N_sim = T_sim / dt;
dt_sub = dt / M;  % pastikan M sudah didefinisikan
time_log = 0:dt:T_sim;

% Dimensi State dan Input
nx = 12; % [x,y,z,phi,theta,psi,vx_inertial,vy_inertial,vz_inertial,p,q,r]
nu = 4;  % [f1, f2, f3, f4]

massa = 0.5;    % Massa (kg)
gravitasi = 9.81;   % Gravitasi (m/s^2)
l = 0.25;   % Panjang lengan (m)
Ixx = 4.85e-3; % Momen inersia
Iyy = 4.85e-3;
Izz = 8.81e-3;

% Hovering thrust per motor (pastikan numeric)
thrust_hover_value = double(massa * gravitasi / 4);  % Force as double
fprintf('Hover thrust per motor: %.3f N\n', thrust_hover_value);

%% 3. Definisikan Model Dinamika Quadrotor yang Konsisten
% Input simbolik
x = MX.sym('x', nx, 1); % State
u = MX.sym('u', nu, 1); % Input (gaya dorong motor)
wind_input = MX.sym('wind_input', 3); % [u_wind; v_wind; w_wind]

% Ekstrak state
px = x(1); py = x(2); pz = x(3);
phi = x(4); theta = x(5); psi = x(6);
vx_inertial = x(7); vy_inertial = x(8); vz_inertial = x(9);
p = x(10); q = x(11); r = x(12);

% Konversi thrust motor ke total force dan torque
F_total = sum(u);
% tau_phi = l * (u(4) - u(2)); % Torsi Roll (Misal u4 naik, u2 turun -> Roll)
% tau_theta = l * (u(3) - u(1)); % Torsi Pitch (Misal u3 naik, u1 turun -> Pitch)  
% Ganti baris 118-119 dengan ini:

tau_phi   = l * ((u(2) + u(3)) - (u(1) + u(4)));     % OK
tau_theta = l * ((u(3) + u(4)) - (u(1) + u(2)));     % tanda dibalik

tau_psi = 0.005 * (-u(1) + u(2) - u(3) + u(4)); 

% Rotation matrices
% R_b_i = rotz(psi) * roty(theta) * rotx(phi);
R_b_i = rotx(phi) * roty(theta) * rotz(psi);

% Inertial-to-Body rotation matrix (transpose)
R_i_b = R_b_i'; % R_i_b = R_b_i^T (orthogonal matrix)

% ===== THRUST FORCE =====
% Thrust dalam body frame (mengarah ke atas sepanjang z-axis body)
thrust_body = [0; 0; -F_total];

% Transform ke inertial frame
thrust_inertial = R_b_i * thrust_body;

% ===== DRAG FORCE (DENGAN WIND) =====
% Kecepatan drone dalam inertial frame
V_inertial = x(7:9); % [vx_inertial; vy_inertial; vz_inertial]

% Transform kecepatan ke body frame
V_body = R_i_b * V_inertial;

% Transform kecepatan angin ke body frame
wind_body = R_i_b * wind_input; % wind_input = [u_wind; v_wind; w_wind]

% Kecepatan relatif dalam body frame (drone relatif terhadap udara)
V_relative_body = V_body - wind_body;

% Koefisien drag dalam body frame
% Cd_x, Cd_y (lateral) biasanya lebih besar dari Cd_z (axial)
Cd = diag([0.5; 0.5; 0.25]); % [Cd_x; Cd_y; Cd_z]

% Gaya drag dalam body frame (proporsional dengan V^2, disederhanakan ke V)
% Untuk model lengkap: F_drag = -0.5 * rho * Cd * A * V^2
% Versi linear: F_drag = -Cd * V
F_drag_body = -Cd * V_relative_body;

% Transform drag force kembali ke inertial frame
F_drag_inertial = R_b_i * F_drag_body;

a_drag_inertial = F_drag_inertial / massa;
% Percepatan dalam inertial frames
ax_inertial = thrust_inertial(1) / massa + a_drag_inertial(1);
ay_inertial = thrust_inertial(2) / massa + a_drag_inertial(2);
az_inertial = thrust_inertial(3) / massa - gravitasi + a_drag_inertial(3);

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
        p_dot; q_dot; r_dot]; % Angular velocity rates

% Fungsi dinamika
f = Function('f', {x, u, wind_input}, {xdot});

% Diskretisasi RK4
X = MX.sym('X', nx, 1);
U = MX.sym('U', nu, 1);
WIND_sym = MX.sym('WIND_sym', 3); % vektor gangguan angin
XDOT = f(X, U, WIND_sym);
k1 = XDOT;
k2 = f(X + dt/2*k1, U, WIND_sym);
k3 = f(X + dt/2*k2, U, WIND_sym);
k4 = f(X + dt*k3, U, WIND_sym);
X_next = X + dt/6*(k1 + 2*k2 + 2*k3 + k4);
F_discrete = Function('F_discrete', {X, U, WIND_sym}, {X_next});

x_next_sub = X + dt_sub/6 * (k1 + 2*k2 + 2*k3 + k4);
F_sub = Function('F_sub', {X, U, WIND_sym}, {x_next_sub});

% Kecepatan rata-rata angin drone (m/s). Jika drone bergerak, ini adalah True Airspeed.
V_w = 0.0000001; 

% Intensitas Turbulensi (Standard Deviation)
sigma_u = 0.2; % Longitudinal (m/s)
sigma_v = 0.15; % Lateral (m/s)
sigma_w = 0.1; % Vertikal (m/s)

% Skala Panjang Turbulensi (Scale Lengths)
% Umumnya Lw=0.5*Lu, Lv=Lu
Lu = 1; % (m)
Lv = Lu;  % (m)
Lw = Lu / 2; % (m)

T_sim_wind = T_sim;
% Generate gangguan angin Von Karman
[time_wind, ug, vg, wg, wind_magnitude] = generate_von_karman_wind( ...
     V_w, sigma_u, sigma_v, sigma_w, Lu, Lv, Lw, dt, T_sim_wind);

% Gabungkan vektor angin
wind_data = [ug, vg, wg];

% Kp = [Kp_x; Kp_y; Kp_z; Kp_phi; Kp_theta; Kp_psi]
Kp = [0.8; 0.8; 6.0; 1.2; 1.2; 4.0];   % Turunkan semua gain sedikit
Kd = [1.5; 1.5; 4.0; 0.8; 0.8; 0.8];   % Kd_z: 12→4 (1/3 dari sebelumnya!)
Ki = [0.01; 0.01; 0.05; 0; 0; 0];      % Turunkan Ki_z juga


% State Integrator PID (Hanya untuk X, Y, Z)
PID_I_state = zeros(3, 1); 

% Struktur Parameter Sistem yang Diperlukan oleh PID
PID_params.massa = massa;
PID_params.gravitasi = gravitasi;
PID_params.l = l;
% Batasan Motor (diambil dari setup NMPC Anda)
PID_params.u_min = 0.0* thrust_hover_value; 
PID_params.u_max = 2.5* thrust_hover_value;



% Simpan Motor Allocation Matrix (dari model dinamika Anda)
k_yaw = 0.005; 
Mixer = [ 1,  1,  1,  1;
         -PID_params.l, PID_params.l, PID_params.l, -PID_params.l;  % KOREKSI: Sesuai tau_phi
         PID_params.l, PID_params.l, -PID_params.l, -PID_params.l;  % KOREKSI: Sesuai tau_theta
         -k_yaw, k_yaw, -k_yaw, k_yaw]; 
PID_params.Mixer_Inv = inv(Mixer);

% Print Mixer dan Mixer_Inv
fprintf('Mixer:\n'); disp(Mixer);
fprintf('Mixer_Inv:\n'); disp(PID_params.Mixer_Inv);

% Test: jika F_z=10N, tau=0, harusnya semua motor = 2.5N
F_test = [10; 0; 0; 0];
u_test_mixer = PID_params.Mixer_Inv * F_test;
fprintf('Test: F_z=10N → u = '); disp(u_test_mixer');
% Expected: [2.5, 2.5, 2.5, 2.5]

x0 = zeros(12, 1);  % Start dari ground, diam
x_current = x0; % <-- Pastikan baris ini ada!
fprintf('x0 = \n'); disp(x0');

% Sesuaikan ukuran wind_data agar pas
if size(wind_data, 1) < N_sim + N
    last_row = wind_data(end, :);
    extra_rows = repmat(last_row, N_sim + N - size(wind_data,1), 1);
    wind_data = [wind_data; extra_rows];
end

% Logging
x_log = zeros(nx, N_sim+1);
u_log = zeros(nu, N_sim);
x_log(:, 1) = x0;
t_sim = 0; % Waktu simulasi awal


% Test PID controller di iterasi pertama
fprintf('=== DEBUG PID CONTROLLER ===\n');
fprintf('Initial State x_current:\n');
fprintf('  Position: [%.3f, %.3f, %.3f]\n', x_current(1:3));
fprintf('  Velocity: [%.3f, %.3f, %.3f]\n', x_current(7:9));
fprintf('  Angles (deg): [%.1f, %.1f, %.1f]\n', rad2deg(x_current(4:6)));

% Target di t=0
x_target_test = QuadrotorReferenceTrajectory4(0);
fprintf('\nTarget State x_target:\n');
fprintf('  Position: [%.3f, %.3f, %.3f]\n', x_target_test(1:3));
fprintf('  Velocity: [%.3f, %.3f, %.3f]\n', x_target_test(7:9));

% Error
error_test = x_target_test(1:3) - x_current(1:3);
fprintf('\nPosition Error: [%.3f, %.3f, %.3f]\n', error_test);

% Test PID
PID_I_test = zeros(3,1);
u_test = pid_controller(x_current, x_target_test, PID_params, Kp, Ki, Kd, PID_I_test);

fprintf('\nPID Output (Motor Thrust):\n');
fprintf('  u1=%.4f N, u2=%.4f N, u3=%.4f N, u4=%.4f N\n', u_test);
fprintf('  Total Thrust: %.4f N\n', sum(u_test));
fprintf('  Weight: %.4f N\n', massa * gravitasi);
fprintf('  Thrust/Weight Ratio: %.2f\n', sum(u_test)/(massa*gravitasi));

fprintf('\nMotor Limits:\n');
fprintf('  u_min: %.4f N (%.1f%% hover)\n', PID_params.u_min, PID_params.u_min/thrust_hover_value*100);
fprintf('  u_max: %.4f N (%.1f%% hover)\n', PID_params.u_max, PID_params.u_max/thrust_hover_value*100);
fprintf('  hover_per_motor: %.4f N\n', thrust_hover_value);

% Check saturasi
num_at_min = sum(abs(u_test - PID_params.u_min) < 0.001);
num_at_max = sum(abs(u_test - PID_params.u_max) < 0.001);
fprintf('\nSaturation Status:\n');
fprintf('  Motors at u_min: %d/4\n', num_at_min);
fprintf('  Motors at u_max: %d/4\n', num_at_max);

if num_at_min == 4
    fprintf('  ⚠️  ALL MOTORS SATURATED AT MIN! Controller cannot increase thrust!\n');
end

% Expected thrust untuk naik
if x_target_test(3) > x_current(3)
    delta_z = x_target_test(3) - x_current(3);
    fprintf('\nExpected Behavior:\n');
    fprintf('  Need to climb %.2f m\n', delta_z);
    fprintf('  Should generate thrust > %.2f N (weight)\n', massa*gravitasi);
    if sum(u_test) < massa*gravitasi
        fprintf('  ❌ INSUFFICIENT THRUST! Drone will fall!\n');
    else
        fprintf('  ✅ Sufficient thrust to climb\n');
    end
end

fprintf('\n=== END DEBUG ===\n\n');
% Loop Simulasi
%% SIMULATION LOOP - FIXED VERSION
for i = 1:N_sim
    
    %% 1. Get Target State at Current Time
    t_ref = t_sim + dt;
    x_target_now = QuadrotorReferenceTrajectory4(t_ref);
    
    %% 2. Update PID Integrator dengan Anti-Windup
    % Error posisi saat ini
    error_pos_current = x_target_now(1:3) - x_current(1:3); 
    
    % Update integrator dengan anti-windup
    PID_I_state = PID_I_state + error_pos_current * dt;
    
    % ANTI-WINDUP: Batasi integrator agar tidak meledak
    I_max = [2.0; 2.0; 5.0];  % Batasan maksimal integrator [X, Y, Z]
    for k = 1:3
        if PID_I_state(k) > I_max(k)
            PID_I_state(k) = I_max(k);
        elseif PID_I_state(k) < -I_max(k)
            PID_I_state(k) = -I_max(k);
        end
    end
    
    % Atau versi singkat:
    % PID_I_state = max(-I_max, min(I_max, PID_I_state));
    
    %% 3. Compute PID Control Input
    u_applied = pid_controller(x_current, x_target_now, PID_params, ...
                                Kp, Ki, Kd, PID_I_state);
    
    % SAFETY: Pastikan u_applied tidak NaN/Inf
    if any(isnan(u_applied)) || any(isinf(u_applied))
        warning('NaN/Inf detected in u_applied at t=%.2f! Using hover thrust.', t_sim);
        u_applied = thrust_hover_value * ones(4,1);
    end
    
    %% 4. Apply Control Input (Simulation with Sub-steps)
    x_next = x_current;
    wind_turbulence = wind_data(i, :)';
    
    % Wind vector (pastikan V_w sudah didefinisikan sebelumnya)
    % Jika V_w terlalu besar, turunkan (misal V_w = 0 untuk test hover)
    current_wind = [V_w; 0; 0] + wind_turbulence;
    
    % Sub-stepping untuk akurasi numerik
    for j = 1:M
        % Simulate one sub-step
        x_next = full(F_sub(x_next, u_applied, current_wind));
        
        % SAFETY CHECK: Batasi sudut untuk hindari singularitas gimbal lock
        % Roll (phi): batasi ke ±60°
        if x_next(4) > pi/3
            x_next(4) = pi/3;
        elseif x_next(4) < -pi/3
            x_next(4) = -pi/3;
        end
        
        % Pitch (theta): batasi ke ±60°
        if x_next(5) > pi/3
            x_next(5) = pi/3;
        elseif x_next(5) < -pi/3
            x_next(5) = -pi/3;
        end
        
        % Yaw (psi): wrap ke [-pi, pi]
        x_next(6) = atan2(sin(x_next(6)), cos(x_next(6)));
        
        % Check untuk nilai abnormal
        if any(isnan(x_next)) || any(isinf(x_next))
            error('Simulation diverged at t=%.2f (step %d, substep %d)!', ...
                  t_sim, i, j);
        end
    end
    
    % Update current state
    x_current = x_next;
    
    %% 5. Logging
    u_log(:, i) = u_applied;
    x_log(:, i+1) = x_current;
    t_sim = t_sim + dt;
    
    %% 6. Debug Output (setiap 50 steps = 5 detik jika dt=0.1)
 if mod(i, 10) == 0 || i <= 5  % Print 5 iterasi pertama + setiap 10 steps
        F_total = sum(u_applied);
        TWR = F_total / (massa * gravitasi);  % Thrust-to-Weight Ratio
        
        fprintf('[t=%.1fs] Z: %.2fm→%.2fm (err=%.2fm) | ', ...
                t_sim, x_current(3), x_target_now(3), x_target_now(3)-x_current(3));
        fprintf('F=%.2fN (TWR=%.2f) | ', F_total, TWR);
        fprintf('motors=[%.2f %.2f %.2f %.2f] | ', u_applied);
        fprintf('angles=[%.1f° %.1f°]\n', rad2deg(x_current(4)), rad2deg(x_current(5)));
        
        % Warning jika thrust tidak cukup
        if TWR < 1.05 && x_target_now(3) > x_current(3)
            fprintf('  ⚠️  TWR < 1.05! Need more thrust to climb!\n');
        end
        
        % Check saturasi
        num_saturated_min = sum(abs(u_applied - PID_params.u_min) < 0.01);
        if num_saturated_min >= 3
            fprintf('  ⚠️  %d motors at u_min! Increase Kp_z or decrease u_min!\n', num_saturated_min);
        end
    end
    
    %% 7. Emergency Stop (jika drone crash ke tanah atau terbang terlalu tinggi)
    if x_current(3) < -1.0  % Crash ke tanah (Z < -1m)
        warning('CRASH DETECTED at t=%.2f! Z=%.2fm', t_sim, x_current(3));
        % Potong simulasi atau lanjutkan dengan thrust max
        break;
    end
    
    if x_current(3) > 50.0  % Terbang terlalu tinggi
        warning('Drone too high at t=%.2f! Z=%.2fm', t_sim, x_current(3));
        break;
    end
    
    % Check jika posisi horizontal terlalu jauh (misal > 100m dari origin)
    if norm(x_current(1:2)) > 100.0
        warning('Drone too far horizontally at t=%.2f! Distance=%.2fm', ...
                t_sim, norm(x_current(1:2)));
        break;
    end
end

% Print final state
fprintf('\n=== SIMULATION COMPLETED ===\n');
fprintf('Final time: %.1f s\n', t_sim);
fprintf('Final position: [%.2f, %.2f, %.2f] m\n', x_current(1), x_current(2), x_current(3));
fprintf('Final angles: [%.1f, %.1f, %.1f] deg\n', ...
        rad2deg(x_current(4)), rad2deg(x_current(5)), rad2deg(x_current(6)));
fprintf('Final target: [%.2f, %.2f, %.2f] m\n', ...
        x_target_now(1), x_target_now(2), x_target_now(3));
error_final = norm(x_target_now(1:3) - x_current(1:3));
fprintf('Final position error: %.3f m\n', error_final);

% Buat matriks untuk menyimpan seluruh trajektori referensi
x_ref_log = zeros(nx, N_sim + 1);

for k = 1:N_sim + 1
    % Panggil fungsi untuk setiap nilai waktu skalar
    x_ref_log(:, k) = QuadrotorReferenceTrajectory4(time_log(k));
end

%% 9. Plot Hasil (Koreksi)
%% 9. Plot Hasil (Dipisah)

% --- WINDOW 1: POSISI (X, Y, Z) ---
figure('Name', 'Posisi Drone (Actual vs Target)');
subplot(3, 1, 1);
plot(time_log, x_log(1, :), 'b', 'LineWidth', 1.5); hold on;
plot(time_log, x_ref_log(1, :), 'r--', 'LineWidth', 1.0);
title('Posisi X');
legend('Actual', 'Target', 'Location', 'best');
xlabel('Waktu (s)');
ylabel('X (m)');
grid on;

subplot(3, 1, 2);
plot(time_log, x_log(2, :), 'b', 'LineWidth', 1.5); hold on;
plot(time_log, x_ref_log(2, :), 'r--', 'LineWidth', 1.0);
title('Posisi Y');
xlabel('Waktu (s)');
ylabel('Y (m)');
grid on;

subplot(3, 1, 3);
plot(time_log, x_log(3, :), 'b', 'LineWidth', 1.5); hold on;
plot(time_log, x_ref_log(3, :), 'r--', 'LineWidth', 1.0);
title('Posisi Z');
xlabel('Waktu (s)');
ylabel('Z (m)');
grid on;

% --- WINDOW 2: SUDUT EULER (ROLL, PITCH, YAW) ---
figure('Name', 'Sudut Euler (Roll, Pitch, Yaw)');
subplot(3, 1, 1);
plot(time_log, rad2deg(x_log(4, :)), 'LineWidth', 1.5);
title('Sudut Roll (\phi)');
xlabel('Waktu (s)');
ylabel('Sudut (deg)');
grid on;

subplot(3, 1, 2);
plot(time_log, rad2deg(x_log(5, :)), 'LineWidth', 1.5);
title('Sudut Pitch (\theta)');
xlabel('Waktu (s)');
ylabel('Sudut (deg)');
grid on;

subplot(3, 1, 3);
plot(time_log, rad2deg(x_log(6, :)), 'LineWidth', 1.5);
title('Sudut Yaw (\psi)');
xlabel('Waktu (s)');
ylabel('Sudut (deg)');
grid on;

% --- WINDOW 3: INPUT MOTOR (THRUST u1-u4) ---
figure('Name', 'Input Motor Thrust');
plot(time_log(1:end-1), u_log(1, :), 'LineWidth', 1.5); hold on;
plot(time_log(1:end-1), u_log(2, :), 'LineWidth', 1.5);
plot(time_log(1:end-1), u_log(3, :), 'LineWidth', 1.5);
plot(time_log(1:end-1), u_log(4, :), 'LineWidth', 1.5);
yline(PID_params.u_max, 'r--', 'LineWidth', 1.5, 'DisplayName', 'Umax'); 
yline(PID_params.u_min, 'r--', 'LineWidth', 1.5, 'DisplayName', 'Umin'); 
title('Input Motor (u1-u4)');
legend('u1 (Motor 1)', 'u2 (Motor 2)', 'u3 (Motor 3)', 'u4 (Motor 4)', 'Location', 'best');
xlabel('Waktu (s)');
ylabel('Dorong (N)');
grid on;

%% 10. Plot Trajektori 2D dan 3D

% --- Plot 2D (Top-Down View: X vs Y) ---
figure;
plot(x_log(1, :), x_log(2, :), 'b-', 'LineWidth', 1.5); hold on;
plot(x_ref_log(1, :), x_ref_log(2, :), 'r--', 'LineWidth', 1.5);
scatter(x_log(1, end), x_log(2, end), 80, 'b', 'filled'); % Posisi Akhir Actual
scatter(x_ref_log(1, end), x_ref_log(2, end), 80, 'r', 'filled'); % Posisi Akhir Target
legend('Actual Trajectory', 'Reference Trajectory', 'Location', 'best');
title('Trajektori Drone 2D (X vs Y)');
xlabel('X (m)');
ylabel('Y (m)');
axis equal; % Penting agar skala X dan Y sama
grid on;

% --- Plot 3D (X, Y, Z) ---
figure;
plot3(x_log(1, :), x_log(2, :), x_log(3, :), 'b-', 'LineWidth', 2.0); hold on;
plot3(x_ref_log(1, :), x_ref_log(2, :), x_ref_log(3, :), 'r--', 'LineWidth', 2.0);

% Tandai Titik Awal (Z=0)
scatter3(x_log(1, 1), x_log(2, 1), x_log(3, 1), 100, 'k', 'o', 'filled'); 
% Tandai Titik Akhir (Z=10m)
scatter3(x_log(1, end), x_log(2, end), x_log(3, end), 100, 'b', 's', 'filled'); 

legend('Actual Trajectory', 'Reference Trajectory', 'Start', 'End', 'Location', 'best');
title('Trajektori Drone 3D (X, Y, Z)');
xlabel('X (m)');
ylabel('Y (m)');
zlabel('Z (m)');
grid on;
view(3); % Atur tampilan ke sudut 3D

function R_x = rotx(t)
    R_x = [1, 0, 0; 0, cos(t), -sin(t); 0, sin(t), cos(t)];
end

function R_y = roty(t)
    R_y = [cos(t), 0, sin(t); 0, 1, 0; -sin(t), 0, cos(t)];
end

function R_z = rotz(t)
    R_z = [cos(t), -sin(t), 0; sin(t), cos(t), 0; 0, 0, 1];
end