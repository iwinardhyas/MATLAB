% 1. Setup CasADi
addpath('C:\Users\DELL\Documents\MATLAB\casadi-3.7.0-windows64-matlab2018b');
import casadi.*

% Clear workspace to avoid variable conflicts
clear variables
clc

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

% Horizon Prediksi NMPC - Multiple Shooting
N = 50;     % Horizon prediksi total
dt = 0.02;  % Time step
M = 5;      % Jumlah shooting intervals per prediction step

% Hovering thrust per motor (pastikan numeric)
thrust_hover_value = double(m * g / 4);  % Force as double
fprintf('Hover thrust per motor: %.3f N\n', thrust_hover_value);
fprintf('Multiple Shooting: N=%d, M=%d shooting intervals\n', N, M);

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

% Percepatan dalam inertial frame
ax_inertial = thrust_inertial(1) / m;
ay_inertial = thrust_inertial(2) / m;
az_inertial = thrust_inertial(3) / m - g;

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
f = Function('f', {x, u}, {xdot});

% Multiple Shooting: Create multiple integration functions dengan sub-intervals
dt_sub = dt / M;  % Sub-interval time step

% Single sub-interval integration (RK4)
X_sub = MX.sym('X_sub', nx, 1);
U_sub = MX.sym('U_sub', nu, 1);
XDOT_sub = f(X_sub, U_sub);
k1 = XDOT_sub;
k2 = f(X_sub + dt_sub/2*k1, U_sub);
k3 = f(X_sub + dt_sub/2*k2, U_sub);
k4 = f(X_sub + dt_sub*k3, U_sub);
X_next_sub = X_sub + dt_sub/6*(k1 + 2*k2 + 2*k3 + k4);
F_sub = Function('F_sub', {X_sub, U_sub}, {X_next_sub});

% Full interval integration dengan multiple shooting
X_start = MX.sym('X_start', nx, 1);
U_full = MX.sym('U_full', nu, 1);
X_current = X_start;
for m = 1:M
    X_current = F_sub(X_current, U_full);
end
F_discrete = Function('F_discrete', {X_start, U_full}, {X_current});

%% 4. Setup Multiple Shooting NMPC Problem
w = {};
J = 0;
g = {};
lbw = []; ubw = []; w0 = [];
lbg = []; ubg = [];

% Storage untuk variables
X_vars = cell(N+1, 1);      % State variables di shooting nodes
U_vars = cell(N, 1);        % Control variables
X_sub_vars = cell(N, M-1);  % Intermediate states dalam setiap interval

% FIXED: Parameter vector definition
% Parameter vector: [current_state; reference_trajectory_horizon]
n_params = nx + nx * (N+1);  % Current state + reference sepanjang horizon (N+1 points)
all_params_sym = MX.sym('all_params', n_params, 1);

% Extract parameters - FIXED indexing
X_initial_param = all_params_sym(1:nx);
X_ref_params = cell(N+1, 1);
for k = 0:N
    start_idx = nx + nx*k + 1;
    end_idx = nx + nx*(k+1);
    X_ref_params{k+1} = all_params_sym(start_idx:end_idx);
end

fprintf('Parameter vector size: %d (should be %d)\n', n_params, nx + nx*(N+1));

%% Multiple Shooting Variables Setup

% Initial state variable (shooting node 0)
X_vars{1} = MX.sym('X_0', nx, 1);
w = {w{:}, X_vars{1}};
lbw = [lbw; -inf*ones(nx,1)];
ubw = [ubw; inf*ones(nx,1)];
w0 = [w0; zeros(nx,1)];

% Initial state constraint
g = {g{:}, X_vars{1} - X_initial_param};
lbg = [lbg; zeros(nx,1)];
ubg = [ubg; zeros(nx,1)];

% Main Multiple Shooting Loop
for k = 0:N-1
    % Control variables untuk interval k
    U_vars{k+1} = MX.sym(['U_' num2str(k)], nu, 1);
    w = {w{:}, U_vars{k+1}};
    lbw = [lbw; 0.2*thrust_hover_value*ones(nu,1)]; % Min 20% hover thrust
    ubw = [ubw; 3.0*thrust_hover_value*ones(nu,1)]; % Max 300% hover thrust
    w0 = [w0; thrust_hover_value*ones(nu,1)]; % Initialize at hover
    
    % Intermediate states dalam shooting interval k
    X_current_shooting = X_sub_vars{k+1, m};
    for m = 1:M-1
        % Intermediate state variables
        X_sub_vars{k+1, m} = MX.sym(['X_' num2str(k) '_sub_' num2str(m)], nx, 1);
        w = {w{:}, X_sub_vars{k+1, m}};
        lbw = [lbw; -inf*ones(nx,1)];
        ubw = [ubw; inf*ones(nx,1)];
        w0 = [w0; zeros(nx,1)];
        
        % Continuity constraint untuk intermediate state
        X_predicted = F_sub(X_current_shooting, U_vars{k+1});
        g = {g{:}, F_sub(X_current_shooting, U_vars{k+1}) - X_sub_vars{k+1, m}};
        lbg = [lbg; zeros(nx,1)];
        ubg = [ubg; zeros(nx,1)];
        
        % Update current state untuk next sub-interval
        X_current_shooting = X_sub_vars{k+1, m};
    end
    
    % Next shooting node state
    X_vars{k+2} = MX.sym(['X_' num2str(k+1)], nx, 1);
    w = {w{:}, X_vars{k+2}};
    lbw = [lbw; -inf*ones(nx,1)];
    ubw = [ubw; inf*ones(nx,1)];
    w0 = [w0; zeros(nx,1)];
    
    % Final continuity constraint untuk shooting interval
    X_predicted_final = F_sub(X_current_shooting, U_vars{k+1});
    g = {g{:}, X_predicted_final - X_vars{k+2}};
    lbg = [lbg; zeros(nx,1)];
    ubg = [ubg; zeros(nx,1)];
    
    % Cost function - Improved tuning untuk Multiple Shooting
    Q = diag([300, 300, 400, ... % px, py, pz - higher weights
              80, 80, 40, ...    % phi, theta, psi
              15, 15, 20, ...    % vx, vy, vz
              1.5, 1.5, 1.5]);  % p, q, r
    R = diag([0.8, 0.8, 0.8, 0.8]); % Control effort penalty
    
    % Stage cost di shooting node k
    J = J + (X_vars{k+1} - X_ref_params{k+1})'*Q*(X_vars{k+1} - X_ref_params{k+1}) + ...
            U_vars{k+1}'*R*U_vars{k+1};
    
    % Optional: Cost pada intermediate states untuk smoothness
    for m = 1:M-1
        Q_sub = 0.1 * Q;  % Reduced weight untuk intermediate states
        % Interpolated reference untuk intermediate state
        alpha = m / M;
        X_ref_interp = (1-alpha) * X_ref_params{k+1} + alpha * X_ref_params{k+2};
        J = J + (X_sub_vars{k+1, m} - X_ref_interp)'*Q_sub*(X_sub_vars{k+1, m} - X_ref_interp);
    end
    
    % Control rate penalty untuk smoothness
    if k > 0
        R_rate = 0.1 * eye(nu);
        J = J + (U_vars{k+1} - U_vars{k})'*R_rate*(U_vars{k+1} - U_vars{k});
    end
end

% Terminal cost
Qf = 3*Q; % Higher terminal weight untuk Multiple Shooting
J = J + (X_vars{N+1} - X_ref_params{N+1})'*Qf*(X_vars{N+1} - X_ref_params{N+1});

% NLP problem
nlp = struct('f', J, 'x', vertcat(w{:}), 'g', vertcat(g{:}), 'p', all_params_sym);

% Display problem dimensions
fprintf('NLP Problem dimensions:\n');
fprintf('  Decision variables: %d\n', length(vertcat(w{:})));
fprintf('  Constraints: %d\n', length(vertcat(g{:})));
fprintf('  Parameters: %d\n', length(all_params_sym));

% Solver options - optimized untuk Multiple Shooting
solver_options = struct;
solver_options.print_time = false;
solver_options.ipopt.max_iter = 300;        % More iterations untuk MS
solver_options.ipopt.tol = 1e-4;
solver_options.ipopt.acceptable_tol = 1e-3;
solver_options.ipopt.linear_solver = 'ma57'; % Better solver untuk larger problems
solver_options.ipopt.hessian_approximation = 'limited-memory';
solver_options.ipopt.print_level = 0;
solver_options.ipopt.mu_strategy = 'adaptive';
solver_options.ipopt.warm_start_init_point = 'yes';
solver_options.ipopt.warm_start_bound_push = 1e-6;
solver_options.ipopt.warm_start_mult_bound_push = 1e-6;

% Fallback ke mumps jika ma57 tidak tersedia
try
    solver = nlpsol('solver', 'ipopt', nlp, solver_options);
catch
    fprintf('ma57 solver not available, falling back to mumps\n');
    solver_options.ipopt.linear_solver = 'mumps';
    solver = nlpsol('solver', 'ipopt', nlp, solver_options);
end

fprintf('Multiple Shooting NMPC setup completed. Problem size: %d variables, %d constraints\n', ...
        length(vertcat(w{:})), length(vertcat(g{:})));



%% 7. Simulation Loop dengan Multiple Shooting - FIXED parameter construction
T_sim = 5;
N_sim = T_sim / dt;
history_x = zeros(nx, N_sim + 1);
history_u = zeros(nu, N_sim);
history_x_ref = zeros(nx, N_sim + 1);
solver_times = zeros(N_sim, 1);
solver_iters = zeros(N_sim, 1);

% Initial state: start closer to first reference point
x_ref_initial = QuadrotorReferenceTrajectory4(0);
current_state = zeros(nx, 1);
current_state(1:3) = x_ref_initial(1:3); % Start at reference position
current_state(3) = max(current_state(3), 0.1); % Ensure minimum altitude
history_x(:, 1) = current_state;

% Initialize warm start dengan reference trajectory
current_time = 0;
X_ref_horizon = generate_reference_horizon(current_time, N, dt, @QuadrotorReferenceTrajectory4);
arg_w0 = generate_ms_warm_start(X_ref_horizon, nx, nu, N, M, thrust_hover_value);

fprintf('Starting Multiple Shooting NMPC simulation...\n');
fprintf('Initial state: [%.3f, %.3f, %.3f]\n', current_state(1:3)');

for i = 1:N_sim
    current_time = (i-1) * dt;
    
    % Generate reference trajectory horizon
    X_ref_horizon = generate_reference_horizon(current_time, N, dt, @QuadrotorReferenceTrajectory4);
    history_x_ref(:, i) = X_ref_horizon(:, 1);
    
    % FIXED: Build parameter vector correctly
    % Format: [current_state; X_ref_horizon(:,1); X_ref_horizon(:,2); ...; X_ref_horizon(:,N+1)]
    actual_params = [current_state; reshape(X_ref_horizon, [], 1)];
    
    % Debug: Check parameter size
    if i == 1
        fprintf('Parameter vector size check:\n');
        fprintf('  current_state size: %d\n', length(current_state));
        fprintf('  X_ref_horizon size: %d x %d = %d\n', size(X_ref_horizon), numel(X_ref_horizon));
        fprintf('  actual_params size: %d\n', length(actual_params));
        fprintf('  Expected params size: %d\n', n_params);
        
        if length(actual_params) ~= n_params
            error('Parameter size mismatch! Got %d, expected %d', length(actual_params), n_params);
        end
    end
    
    % Solve Multiple Shooting NMPC
    tic;
    try
        sol = solver('x0', arg_w0, 'lbx', lbw, 'ubx', ubw, ...
                     'lbg', lbg, 'ubg', ubg, 'p', actual_params);
        
        % Extract optimal control (first control input)
        opt_w = full(sol.x);
        u_optimal = opt_w(nx + 1 : nx + nu);
        
        % Solver statistics
        solver_times(i) = toc;
        if isfield(sol, 'stats')
            solver_iters(i) = sol.stats.iter_count;
            status = char(sol.stats.return_status);
            if ~strcmp(status, 'Solve_Succeeded') && mod(i, 20) == 0
                fprintf('Warning: Solver status at step %d: %s\n', i, status);
            end
        end
        
    catch ME
        solver_times(i) = toc;
        fprintf('Solver failed at step %d: %s\n', i, ME.message);
        % Use hover thrust as fallback
        u_optimal = thrust_hover_value * ones(nu, 1);
        opt_w = arg_w0; % Keep previous solution for warm start
    end
    
    history_u(:, i) = u_optimal;
    
    % Simulate system forward
    current_state = full(F_discrete(current_state, u_optimal));
    history_x(:, i+1) = current_state;
    
    % Advanced warm start untuk Multiple Shooting
    arg_w0 = shift_ms_solution(opt_w, nx, nu, N, M, X_ref_horizon, thrust_hover_value);
    
    % Progress display dengan detailed analysis
    if mod(i, 20) == 0
        pos_error = norm(current_state(1:3) - X_ref_horizon(1:3, 1));
        vel_error = norm(current_state(7:9) - X_ref_horizon(7:9, 1));
        att_error = norm(current_state(4:6) - X_ref_horizon(4:6, 1));
        
        fprintf('Step %d/%d, Pos_err: %.3f, Vel_err: %.3f, Att_err: %.3f\n', ...
                i, N_sim, pos_error, vel_error, att_error);
        fprintf('           Pos: [%.2f, %.2f, %.2f], Ref: [%.2f, %.2f, %.2f]\n', ...
                current_state(1:3)', X_ref_horizon(1:3, 1)');
        fprintf('           Solver: %.1fms, %d iters, Thrust: [%.2f, %.2f, %.2f, %.2f] N\n', ...
                solver_times(i)*1000, solver_iters(i), u_optimal');
    end
end

% Final reference point
history_x_ref(:, N_sim + 1) = QuadrotorReferenceTrajectory4(T_sim);

fprintf('Multiple Shooting NMPC simulation completed!\n');
fprintf('Average solver time: %.2f ms\n', mean(solver_times)*1000);
fprintf('Average iterations: %.1f\n', mean(solver_iters));

% Performance analysis
pos_errors = sqrt(sum((history_x(1:3, 1:end-1) - history_x_ref(1:3, 1:end-1)).^2, 1));
fprintf('Tracking performance - RMS position error: %.4f m\n', rms(pos_errors));
fprintf('                      Max position error: %.4f m\n', max(pos_errors));

%% 8. Enhanced Plotting
fprintf('PlotTrajectory function not found. Creating enhanced plots...\n');

time_vec = 0:dt:T_sim;
time_u = 0:dt:(T_sim-dt);

figure('Name', 'Multiple Shooting NMPC Results', 'Position', [50 50 1400 900]);

% Position tracking
subplot(3,4,1);
plot(time_vec, history_x(1,:), 'b-', 'LineWidth', 2); hold on;
plot(time_vec, history_x_ref(1,:), 'r--', 'LineWidth', 1.5);
xlabel('Time (s)'); ylabel('X Position (m)'); title('X Position Tracking');
legend('Actual', 'Reference', 'Location', 'best'); grid on;

subplot(3,4,2);
plot(time_vec, history_x(2,:), 'b-', 'LineWidth', 2); hold on;
plot(time_vec, history_x_ref(2,:), 'r--', 'LineWidth', 1.5);
xlabel('Time (s)'); ylabel('Y Position (m)'); title('Y Position Tracking');
legend('Actual', 'Reference', 'Location', 'best'); grid on;

subplot(3,4,3);
plot(time_vec, history_x(3,:), 'b-', 'LineWidth', 2); hold on;
plot(time_vec, history_x_ref(3,:), 'r--', 'LineWidth', 1.5);
xlabel('Time (s)'); ylabel('Z Position (m)'); title('Z Position Tracking');
legend('Actual', 'Reference', 'Location', 'best'); grid on;

% 3D trajectory
subplot(3,4,4);
plot3(history_x(1,:), history_x(2,:), history_x(3,:), 'b-', 'LineWidth', 2); hold on;
plot3(history_x_ref(1,:), history_x_ref(2,:), history_x_ref(3,:), 'r--', 'LineWidth', 1.5);
xlabel('X (m)'); ylabel('Y (m)'); zlabel('Z (m)'); title('3D Trajectory');
legend('Actual', 'Reference', 'Location', 'best'); grid on; axis equal;

% Velocity tracking
subplot(3,4,5);
plot(time_vec, history_x(7,:), 'b-', 'LineWidth', 1.5); hold on;
plot(time_vec, history_x_ref(7,:), 'r--', 'LineWidth', 1.5);
xlabel('Time (s)'); ylabel('Vx (m/s)'); title('X Velocity');
legend('Actual', 'Reference'); grid on;

subplot(3,4,6);
plot(time_vec, history_x(8,:), 'b-', 'LineWidth', 1.5); hold on;
plot(time_vec, history_x_ref(8,:), 'r--', 'LineWidth', 1.5);
xlabel('Time (s)'); ylabel('Vy (m/s)'); title('Y Velocity');
legend('Actual', 'Reference'); grid on;

subplot(3,4,7);
plot(time_vec, history_x(9,:), 'b-', 'LineWidth', 1.5); hold on;
plot(time_vec, history_x_ref(9,:), 'r--', 'LineWidth', 1.5);
xlabel('Time (s)'); ylabel('Vz (m/s)'); title('Z Velocity');
legend('Actual', 'Reference'); grid on;

% Attitude
subplot(3,4,8);
plot(time_vec, rad2deg(history_x(4:6,:)'));
xlabel('Time (s)'); ylabel('Angle (degrees)'); title('Attitude');
legend('\phi (roll)', '\theta (pitch)', '\psi (yaw)'); grid on;

% Control inputs
subplot(3,4,9);
plot(time_u, history_u');
xlabel('Time (s)'); ylabel('Thrust (N)'); title('Motor Thrusts');
legend('Motor 1', 'Motor 2', 'Motor 3', 'Motor 4'); grid on;

% Tracking errors
subplot(3,4,10);
pos_errors = sqrt(sum((history_x(1:3, 1:end-1) - history_x_ref(1:3, 1:end-1)).^2, 1));
plot(time_u, pos_errors, 'r-', 'LineWidth', 1.5);
xlabel('Time (s)'); ylabel('Position Error (m)'); title('Position Tracking Error');
grid on;

% Solver performance
subplot(3,4,11);
plot(time_u, solver_times*1000, 'g-', 'LineWidth', 1.5);
xlabel('Time (s)'); ylabel('Solver Time (ms)'); title('Computational Performance');
grid on;

subplot(3,4,12);
plot(time_u, solver_iters, 'k-', 'LineWidth', 1.5);
xlabel('Time (s)'); ylabel('Iterations'); title('Solver Iterations');
grid on;

%% Support Functions untuk Multiple Shooting

%% 5. Enhanced Reference Trajectory Generation
function X_ref_horizon = generate_reference_horizon(current_time, N, dt, traj_func)
    X_ref_horizon = zeros(12, N+1);
    for k = 0:N
        future_time = current_time + k*dt;
        X_ref_horizon(:, k+1) = traj_func(future_time);
    end
end

%% 6. Improved Warm Start untuk Multiple Shooting
function w0_ms = generate_ms_warm_start(X_ref_horizon, nx, nu, N, M, thrust_hover)
    w0_ms = [];
    
    % Initial state
    w0_ms = [w0_ms; X_ref_horizon(:, 1)];
    
    for k = 0:N-1
        % Control input (feedforward dari reference)
        if k < size(X_ref_horizon, 2) - 1
            vel_ref = X_ref_horizon(7:9, k+1);
            acc_ref = (X_ref_horizon(7:9, k+2) - vel_ref) / 0.02;
            f_total_ref = 0.5 * (norm(acc_ref + [0;0;9.81]) + 4*thrust_hover);
            u_ff = min(max(f_total_ref/4, 0.2*thrust_hover), 3.0*thrust_hover);
            w0_ms = [w0_ms; u_ff*ones(4,1)];
        else
            w0_ms = [w0_ms; thrust_hover*ones(4,1)];
        end
        
        % Intermediate states (linear interpolation)
        for m = 1:M-1
            if k+1 < size(X_ref_horizon, 2)
                alpha = m / M;
                x_interp = (1-alpha) * X_ref_horizon(:, k+1) + alpha * X_ref_horizon(:, k+2);
                w0_ms = [w0_ms; x_interp];
            else
                w0_ms = [w0_ms; X_ref_horizon(:, end)];
            end
        end
        
        % Next shooting node
        if k+1 < size(X_ref_horizon, 2)
            w0_ms = [w0_ms; X_ref_horizon(:, k+2)];
        else
            w0_ms = [w0_ms; X_ref_horizon(:, end)];
        end
    end
end

function w_shifted = shift_ms_solution(w_opt, nx, nu, N, M, X_ref_horizon, thrust_hover_value)
    % Advanced shifting untuk Multiple Shooting solution
    w_shifted = [];
    
    % Parse current solution
    offset = 0;
    
    % Skip first state (will be replaced dengan current state)
    offset = offset + nx;
    
    % Extract and shift solution
    for k = 1:N-1
        % Control input k -> becomes control input k-1
        u_k = w_opt(offset + 1 : offset + nu);
        offset = offset + nu;
        
        % Intermediate states
        for m = 1:M-1
            x_sub_km = w_opt(offset + 1 : offset + nx);
            offset = offset + nx;
        end
        
        % Next shooting node
        x_kplus1 = w_opt(offset + 1 : offset + nx);
        offset = offset + nx;
        
        % Add to shifted solution
        if k == 1
            % First state akan di-set sebagai current state
            w_shifted = [w_shifted; X_ref_horizon(:, 1)]; % Will be overwritten
        end
        
        w_shifted = [w_shifted; u_k];
        
        % Add intermediate states (recompute atau shift)
        for m = 1:M-1
            alpha = m / M;
            if k < N-1
                x_interp = (1-alpha) * x_kplus1 + alpha * X_ref_horizon(:, k+2);
            else
                x_interp = x_kplus1;
            end
            w_shifted = [w_shifted; x_interp];
        end
        
        w_shifted = [w_shifted; x_kplus1];
    end
    
    % Add final control dan state (extrapolate)
    if length(w_opt) >= offset + nu
        u_last = w_opt(offset - nu - nx*(M-1) + 1 : offset - nx*(M-1));
    else
        u_last = thrust_hover_value * ones(nu, 1);
    end
    w_shifted = [w_shifted; u_last];
    
    % Final intermediate states
    for m = 1:M-1
        w_shifted = [w_shifted; X_ref_horizon(:, end)];
    end
    
    % Final state
    w_shifted = [w_shifted; X_ref_horizon(:, end)];
    
    % Ensure correct length
    expected_length = nx + N*(nu + nx*(M-1) + nx);
    if length(w_shifted) ~= expected_length
        fprintf('Warning: Shifted solution length mismatch. Expected %d, got %d\n', ...
                expected_length, length(w_shifted));
        % Fallback: regenerate warm start
        w_shifted = generate_ms_warm_start(X_ref_horizon, nx, nu, N, M, thrust_hover_value);
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