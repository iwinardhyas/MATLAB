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
g = 9.81;   % Gravitasi (m/s^2)
l = 0.25;   % Panjang lengan (m)
Ixx = 4.85e-3; % Momen inersia
Iyy = 4.85e-3;
Izz = 8.81e-3;

% Pastikan semua parameter adalah scalar numeric
assert(isnumeric(m) && isscalar(m), 'Mass m must be numeric scalar');
assert(isnumeric(g) && isscalar(g), 'Gravity g must be numeric scalar');

% Dimensi State dan Input
nx = 12; % [x,y,z,phi,theta,psi,vx_inertial,vy_inertial,vz_inertial,p,q,r]
nu = 4;  % [f1, f2, f3, f4]

W = [0.5; 0; 0];

M = 2;
% Horizon Prediksi NMPC
N = 30; % Kurangi horizon untuk konvergensi lebih baik
dt = 0.01; % Time step lebih kecil
dt_sub = dt / M;  % pastikan M sudah didefinisikan

% Hovering thrust per motor (pastikan numeric)
thrust_hover_value = double(m * g / 4);  % Force as double
fprintf('Hover thrust per motor: %.3f N\n', thrust_hover_value);

%% 3. Definisikan Model Dinamika Quadrotor yang Konsisten
% Input simbolik
x = MX.sym('x', nx, 1); % State
u = MX.sym('u', nu, 1); % Input (gaya dorong motor)

% Ekstrak state
px = x(1); py = x(2); pz = x(3);
phi = x(4); theta = x(5); psi = x(6);
vx_inertial = x(7); vy_inertial = x(8); vz_inertial = x(9);
p = x(10); q = x(11); r = x(12);

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

% === Tambahkan Angin Eksternal ===
% Angin W = [Wx; Wy; Wz] (input tambahan, bukan state)
Wx = W(1); Wy = W(2); Wz = W(3);

% Kecepatan relatif drone terhadap udara
v_rel = [vx_inertial; vy_inertial; vz_inertial] - [Wx; Wy; Wz];

% Drag force (linear model)
C_d = 1; % koefisien drag (bisa disesuaikan)
drag_force = -C_d * v_rel; % bekerja di inertial frame

% Percepatan dalam inertial frame
ax_inertial = (thrust_inertial(1) + drag_force(1)) / m;
ay_inertial = (thrust_inertial(2) + drag_force(2)) / m;
az_inertial = (thrust_inertial(3) + drag_force(3)) / m - g;

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
for k = 1:N
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
for k = 0:N-1
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
T_sim = 5;
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
    
    if i == 1
        actual_wind = zeros(3,1);
    else
        wind_process_noise = 0.5 * randn(3,1);
        actual_wind = history_wind_actual(:, i-1) + wind_process_noise;
    end
    history_wind_actual(:, i) = actual_wind;
    disp("Actual wind: " + num2str(actual_wind'))
    
    current_wind_estimate = actual_wind;
    
    % Get reference trajectory
    x_ref_at_current_time = QuadrotorReferenceTrajectory1(current_time);
    history_x_ref(:, i) = x_ref_at_current_time;
    
    W_val = repmat(current_wind_estimate, N, 1);  % misalnya estimasi konstan
    
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
    
    history_u(:, i) = u_optimal;
    
    % Simulate system forward
    current_state = full(F_discrete(current_state, u_optimal, actual_wind));
    history_x(:, i+1) = current_state;
    
    % Update warm start
    arg_w0 = shift_solution(opt_w, nx, nu, N, M);
    
    % Progress display with error analysis
    if mod(i, 20) == 0
        pos_error = norm(current_state(1:3) - x_ref_at_current_time(1:3));
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

fprintf('PlotTrajectory function not found. Creating basic plots...\n');

% % === Pastikan t_history sesuai ===
% t_history = (0:N_sim-1) * dt;
% 
% % === Ambil data posisi dan referensi ===
% x_actual = history_x(1, 1:N_sim);
% y_actual = history_x(2, 1:N_sim);
% z_actual = history_x(3, 1:N_sim);
% 
% x_ref = history_x_ref(1, 1:N_sim);
% y_ref = history_x_ref(2, 1:N_sim);
% z_ref = history_x_ref(3, 1:N_sim);
% 
% % === Plot posisi X, Y, Z ===
% figure;
% subplot(2,3,1);
% plot(t_history, x_actual, 'b', t_history, x_ref, 'r--'); grid on;
% xlabel('Time [s]'); ylabel('X Position [m]'); title('X Position');
% legend('Actual','Reference');
% 
% subplot(2,3,2);
% plot(t_history, y_actual, 'b', t_history, y_ref, 'r--'); grid on;
% xlabel('Time [s]'); ylabel('Y Position [m]'); title('Y Position');
% legend('Actual','Reference');
% 
% subplot(2,3,3);
% plot(t_history, z_actual, 'b', t_history, z_ref, 'r--'); grid on;
% xlabel('Time [s]'); ylabel('Z Position [m]'); title('Z Position');
% legend('Actual','Reference');
% 
% % === 3D Trajectory ===
% subplot(2,3,4);
% plot3(x_actual, y_actual, z_actual, 'b', x_ref, y_ref, z_ref, 'r--'); grid on;
% xlabel('X [m]'); ylabel('Y [m]'); zlabel('Z [m]');
% title('3D Trajectory');
% legend('Actual','Reference');
% axis equal;
% 
% % === Motor thrust ===
% subplot(2,3,5);
% plot(t_history, history_u(:, 1:N_sim)');
% xlabel('Time [s]'); ylabel('Thrust [N]');
% title('Motor Thrusts'); grid on;
% legend('Motor 1','Motor 2','Motor 3','Motor 4');
% 
% % === Orientation ===
% subplot(2,3,6);
% phi = rad2deg(history_x(4, 1:N_sim));
% theta = rad2deg(history_x(5, 1:N_sim));
% psi = rad2deg(history_x(6, 1:N_sim));
% plot(t_history, phi, t_history, theta, t_history, psi);
% xlabel('Time [s]'); ylabel('Angle [deg]');
% title('Orientation'); grid on;
% legend('\phi (roll)','\theta (pitch)','\psi (yaw)');
% 
% % === Plot gangguan angin ===
% figure;
% plot(t_history, history_wind_actual(:,1:N_sim));
% xlabel('Time [s]'); ylabel('Wind [m/s]');
% title('Actual Wind Disturbance');
% legend('Wx','Wy','Wz'); grid on;
% 
% % === Plot tracking error posisi ===
% tracking_error = vecnorm(history_x(1:3,1:N_sim) - history_x_ref(1:3,1:N_sim));
% figure;
% plot(t_history, tracking_error);
% xlabel('Time [s]'); ylabel('Error [m]');
% title('Position Tracking Error');
% grid on;

% === PERBAIKAN PLOTTING CODE ===

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

% === Print statistik untuk debugging ===
fprintf('\n=== PLOTTING DEBUG INFO ===\n');
fprintf('X range: %.3f to %.3f\n', min(x_actual), max(x_actual));
fprintf('Y range: %.3f to %.3f\n', min(y_actual), max(y_actual));  
fprintf('Z range: %.3f to %.3f\n', min(z_actual), max(z_actual));
fprintf('Z reference range: %.3f to %.3f\n', min(z_ref), max(z_ref));
fprintf('Z error mean: %.6f, std: %.6f\n', mean(z_error), std(z_error));


%% Support Functions
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