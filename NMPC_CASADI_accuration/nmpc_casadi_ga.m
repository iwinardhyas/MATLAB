function total_error = nmpc_casadi_ga(Q_vec, R_vec)

    addpath('C:\Users\DELL\Documents\MATLAB\casadi-3.7.0-windows64-matlab2018b');
    import casadi.*

    % Clear all variables except input
    clearvars -except Q_vec R_vec
%     clc

    Q = diag(Q_vec);
    R = diag(R_vec);


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

    % Horizon Prediksi NMPC
    N = 10; % Kurangi horizon untuk konvergensi lebih baik
    dt = 0.05; % Time step lebih kecil

    % Hovering thrust per motor (pastikan numeric)
    thrust_hover_value = double(m * g / 4);  % Force as double
%     fprintf('Hover thrust per motor: %.3f N\n', thrust_hover_value);

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

    % Diskretisasi RK4
    X = MX.sym('X', nx, 1);
    U = MX.sym('U', nu, 1);
    XDOT = f(X, U);
    k1 = XDOT;
    k2 = f(X + dt/2*k1, U);
    k3 = f(X + dt/2*k2, U);
    k4 = f(X + dt*k3, U);
    X_next = X + dt/6*(k1 + 2*k2 + 2*k3 + k4);
    F_discrete = Function('F_discrete', {X, U}, {X_next});

    %% 4. Setup NMPC Problem
    w = {};
    J = 0;
    g = {};
    lbw = []; ubw = []; w0 = [];
    lbg = []; ubg = [];

    X_vars = cell(N+1, 1);
    U_vars = cell(N, 1);

    % Parameter vector: [current_state; reference_state]
    n_params = 2 * nx;
    all_params_sym = MX.sym('all_params', n_params, 1);
    X_initial_param = all_params_sym(1:nx);
    X_ref_param = all_params_sym(nx+1:end);

    % Initial state variable
    X_vars{1} = MX.sym('X_0', nx, 1);
    w = {w{:}, X_vars{1}};
    lbw = [lbw; -inf*ones(nx,1)];
    ubw = [ubw; inf*ones(nx,1)];
    w0 = [w0; zeros(nx,1)];

    % Initial state constraint
    g = {g{:}, X_vars{1} - X_initial_param};
    lbg = [lbg; zeros(nx,1)];
    ubg = [ubg; zeros(nx,1)];

    % Main horizon loop
    for k = 0:N-1
        % Control variables
        U_vars{k+1} = MX.sym(['U_' num2str(k)], nu, 1);
        w = {w{:}, U_vars{k+1}};
        lbw = [lbw; 0.1*thrust_hover_value*ones(nu,1)]; % Min 10% hover thrust
        ubw = [ubw; 5*thrust_hover_value*ones(nu,1)];   % Max 300% hover thrust
        w0 = [w0; thrust_hover_value*ones(nu,1)]; % Initialize at hover

        % Next state variables
        X_vars{k+2} = MX.sym(['X_' num2str(k+1)], nx, 1);
        w = {w{:}, X_vars{k+2}};
        lbw = [lbw; -inf*ones(nx,1)];
        ubw = [ubw; inf*ones(nx,1)];
        w0 = [w0; zeros(nx,1)];

        % Cost function - CONSERVATIVE tuning untuk stabilitas
        % Position tracking dengan weight yang reasonable
    %     Q = diag([100, 100, 100, ... % px, py, pz
    %                    10, 10, 10, ...   % phi, theta, psi
    %                    1, 1, 1, ...      % vx, vy, vz
    %                    0.1, 0.1, 0.1]);  % p, q, r
    %     R = diag([0.1, 0.1, 0.1, 0.1]); % Bobot upaya kontrol

    %     Q = diag([200, 250, 250, ...   % px, py, pz (utamakan lateral & vertical lebih tinggi)
    %                100, 80, 50, ...     % phi, theta, psi (lebih penalize attitude untuk respons cepat)
    %                15, 20, 10, ...      % vx, vy, vz
    %                0.5, 0.5, 0.5]);    % p, q, r (rate, cukup rendah)
    % 
    %     R = diag([0.5, 0.5, 0.5, 0.5]); % penalti kontrol kecil agar NMPC agresif



        J = J + (X_vars{k+1} - X_ref_param)'*Q*(X_vars{k+1} - X_ref_param) + ...
                U_vars{k+1}'*R*U_vars{k+1};

        % Dynamics constraint
        g = {g{:}, F_discrete(X_vars{k+1}, U_vars{k+1}) - X_vars{k+2}};
        lbg = [lbg; zeros(nx,1)];
        ubg = [ubg; zeros(nx,1)];
    end

    % Terminal cost
    Qf = 2*Q; % Higher terminal weight
    J = J + (X_vars{N+1} - X_ref_param)'*Qf*(X_vars{N+1} - X_ref_param);

    % NLP problem
    nlp = struct('f', J, 'x', vertcat(w{:}), 'g', vertcat(g{:}), 'p', all_params_sym);

    % Solver options - lebih konservatif untuk stabilitas
    solver_options = struct;
    solver_options.print_time = false;
    solver_options.ipopt.max_iter = 100;
    solver_options.ipopt.tol = 1e-3;
    solver_options.ipopt.acceptable_tol = 1e-2;
    solver_options.ipopt.linear_solver = 'mumps';
    solver_options.ipopt.hessian_approximation = 'limited-memory';
    solver_options.ipopt.print_level = 0;
    solver_options.ipopt.mu_strategy = 'adaptive';

    solver = nlpsol('solver', 'ipopt', nlp, solver_options);

    %% 5. Simulation Loop
    T_sim = 2;
    N_sim = T_sim / dt;
    history_x = zeros(nx, N_sim + 1);
    history_u = zeros(nu, N_sim);
    history_x_ref = zeros(nx, N_sim + 1);

    % Initial state: start closer to first reference point
    x_ref_initial = QuadrotorReferenceTrajectory4(0);
    current_state = zeros(nx, 1);
    current_state(1:3) = x_ref_initial(1:3); % Start at reference position
    current_state(3) = max(current_state(3), 0.1); % Ensure minimum altitude
    history_x(:, 1) = current_state;

    % Initialize warm start
    arg_w0 = w0;

    fprintf('Starting NMPC simulation...\n');
    fprintf('Initial state: [%.3f, %.3f, %.3f]\n', current_state(1:3)');
    fprintf('Target trajectory: debug_hover to 2m altitude\n');

    for i = 1:N_sim
        current_time = (i-1) * dt;

        % Get reference trajectory
        x_ref_at_current_time = QuadrotorReferenceTrajectory4(current_time);
        history_x_ref(:, i) = x_ref_at_current_time;

        % Build parameter vector
        actual_params = [current_state; x_ref_at_current_time];

        % Solve NMPC
        try
            sol = solver('x0', arg_w0, 'lbx', lbw, 'ubx', ubw, ...
                         'lbg', lbg, 'ubg', ubg, 'p', actual_params);

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
        current_state = full(F_discrete(current_state, u_optimal));
        history_x(:, i+1) = current_state;

        % Update warm start
        arg_w0 = shift_solution(opt_w, nx, nu, N);

        % Progress display with error analysis
%         if mod(i, 20) == 0
%             pos_error = norm(current_state(1:3) - x_ref_at_current_time(1:3));
%             fprintf('Step %d/%d, Pos: [%.2f, %.2f, %.2f], Ref: [%.2f, %.2f, %.2f], Error: %.2f\n', ...
%                     i, N_sim, current_state(1), current_state(2), current_state(3), ...
%                     x_ref_at_current_time(1), x_ref_at_current_time(2), x_ref_at_current_time(3), pos_error);
%             fprintf('           Thrust: [%.2f, %.2f, %.2f, %.2f] N\n', u_optimal');
%         end
    end

        % Final reference point
        history_x_ref(:, N_sim + 1) = QuadrotorReferenceTrajectory4(T_sim);

        fprintf('Simulation completed!\n');

        % Hitung error tracking untuk dijadikan cost
        error_tracking = history_x - history_x_ref;  % selisih state tracking
        total_error = sum(vecnorm(error_tracking).^2);  % atau gunakan RMSE
end

%% Support Functions
function w_shifted = shift_solution(w_opt, nx, nu, N)
    extracted_x = cell(N+1, 1);
    extracted_u = cell(N, 1);
    
    offset = 0;
    extracted_x{1} = w_opt(offset + 1 : offset + nx);
    offset = offset + nx;
    
    for k = 0:N-1
        extracted_u{k+1} = w_opt(offset + 1 : offset + nu);
        offset = offset + nu;
        extracted_x{k+2} = w_opt(offset + 1 : offset + nx);
        offset = offset + nx;
    end
    
    w_shifted = [];
    w_shifted = [w_shifted; extracted_x{2}];
    
    for k = 0:N-2
        w_shifted = [w_shifted; extracted_u{k+2}];
        w_shifted = [w_shifted; extracted_x{k+3}];
    end
    
    w_shifted = [w_shifted; extracted_u{N}];
    w_shifted = [w_shifted; extracted_x{N+1}];
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

