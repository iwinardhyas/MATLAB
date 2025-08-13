clear; clc;

%% Parameter Waktu
dt = 0.01;
T  = 5;
N  = T/dt;

%% Parameter Quadrotor
g_val   = 9.81;
l_val   = 0.2;
Ixx_val = 0.02;
Iyy_val = 0.02;
Izz_val = 0.04;
m       = 1.0;

%% State: [x y z phi theta psi vx vy vz p q r]
nx = 12;
x_true = zeros(nx,1);
x_true(3) = 1;  % mulai di ketinggian 1 m

%% Input: [Fx Fy Fz tau_phi tau_theta tau_psi]
nu = 6;
u_input = zeros(nu,N);
u_input(3,:) = m*g_val; % Hover thrust

%% Measurement: ambil posisi + kecepatan linier
ny = 6; % [x y z vx vy vz]
R = diag([0.05 0.05 0.05 0.02 0.02 0.02].^2);

%% Noise proses
Q = diag([0.01*ones(1,3), 0.001*ones(1,3), 0.05*ones(1,3), 0.01*ones(1,3)].^2);

%% Parameter UKF
alpha = 1e-3; beta = 2; kappa = 0;

%% Inisialisasi UKF
ukf_x_est = zeros(nx,1);
ukf_P = eye(nx);

%% Logging
history_true = zeros(nx,N);
history_est  = zeros(nx,N);

%% Simulasi
for k = 1:N
    % --- Dinamika Sebenarnya ---
    x_true = quad_dynamics_for_ukf(x_true, u_input(:,k), dt, g_val, l_val, Ixx_val, Iyy_val, Izz_val);
    x_true = x_true + sqrt(Q)*randn(nx,1); % Tambah noise proses
    history_true(:,k) = x_true;

    % --- Pengukuran ---
    measurement = [x_true(1:3); x_true(7:9)] + sqrt(R)*randn(ny,1);

    % --- Prediksi UKF ---
    [x_pred, P_pred] = ukf_predict3(ukf_x_est, ukf_P, u_input(:,k), ...
        @quad_dynamics_for_ukf, Q, dt, g_val, alpha, kappa, beta, l_val, Ixx_val, Iyy_val, Izz_val);

    % --- Update UKF ---
    [ukf_x_est, ukf_P] = ukf_update3(x_pred, P_pred, measurement, ...
        @(x)[x(1:3); x(7:9)], R, alpha, kappa, beta);

    history_est(:,k) = ukf_x_est;
end

%% Plot Hasil
figure;
subplot(3,1,1); plot(0:dt:T-dt, history_true(1,:), 'k', 0:dt:T-dt, history_est(1,:), 'r--'); ylabel('X [m]'); legend('True','UKF');
subplot(3,1,2); plot(0:dt:T-dt, history_true(2,:), 'k', 0:dt:T-dt, history_est(2,:), 'r--'); ylabel('Y [m]');
subplot(3,1,3); plot(0:dt:T-dt, history_true(3,:), 'k', 0:dt:T-dt, history_est(3,:), 'r--'); ylabel('Z [m]'); xlabel('Time [s]');

figure;
subplot(3,1,1); plot(0:dt:T-dt, history_true(7,:), 'k', 0:dt:T-dt, history_est(7,:), 'r--'); ylabel('Vx [m/s]');
subplot(3,1,2); plot(0:dt:T-dt, history_true(8,:), 'k', 0:dt:T-dt, history_est(8,:), 'r--'); ylabel('Vy [m/s]');
subplot(3,1,3); plot(0:dt:T-dt, history_true(9,:), 'k', 0:dt:T-dt, history_est(9,:), 'r--'); ylabel('Vz [m/s]'); xlabel('Time [s]');


function x_next = quad_dynamics_for_ukf(x, u, dt, g_val, l_val, Ixx_val, Iyy_val, Izz_val)
    % x = [x y z phi theta psi vx vy vz p q r]
    % u = [Fx Fy Fz tau_phi tau_theta tau_psi]

    m = 1.0;

    % Ekstraksi state
    phi   = x(4); theta = x(5); psi   = x(6);
    vx    = x(7); vy    = x(8); vz    = x(9);
    p     = x(10); q     = x(11); r     = x(12);

    % Translasi
    x_next(1,1) = x(1) + dt*vx;
    x_next(2,1) = x(2) + dt*vy;
    x_next(3,1) = x(3) + dt*vz;

    % Rotasi
    x_next(4,1) = phi   + dt*p;
    x_next(5,1) = theta + dt*q;
    x_next(6,1) = psi   + dt*r;

    % Dinamika kecepatan linier
    ax = u(1)/m;
    ay = u(2)/m;
    az = (u(3)/m) - g_val;

    x_next(7,1) = vx + dt*ax;
    x_next(8,1) = vy + dt*ay;
    x_next(9,1) = vz + dt*az;

    % Dinamika kecepatan angular
    x_next(10,1) = p + dt*(u(4)/Ixx_val);
    x_next(11,1) = q + dt*(u(5)/Iyy_val);
    x_next(12,1) = r + dt*(u(6)/Izz_val);
end

function [x_pred, P_pred] = ukf_predict3(x_est, P, u, ukf_dynamics_fcn, Q, dt, g_val, alpha, kappa, beta, l_val, Ixx_val, Iyy_val, Izz_val)
    nx = numel(x_est);
    lambda = alpha^2*(nx+kappa)-nx;
    Wm = [lambda/(nx+lambda); 0.5/(nx+lambda)*ones(2*nx,1)];
    Wc = Wm; Wc(1) = Wc(1) + (1-alpha^2+beta);

    % Sigma Points
    P = 0.5*(P+P'); % Simetris
    sqrtP = chol((nx+lambda)*P,'lower');
    X_sigma = [x_est, x_est+sqrtP, x_est-sqrtP];

    % Propagasi sigma points
    X_sigma_pred = zeros(nx,2*nx+1);
    for i=1:2*nx+1
        X_sigma_pred(:,i) = ukf_dynamics_fcn(X_sigma(:,i), u, dt, g_val, l_val, Ixx_val, Iyy_val, Izz_val);
    end

    % Mean
    x_pred = X_sigma_pred*Wm;

    % Covariance
    P_pred = Q;
    for i=1:2*nx+1
        diff = X_sigma_pred(:,i)-x_pred;
        P_pred = P_pred + Wc(i)*(diff*diff');
    end
end

function [x_est, P] = ukf_update3(x_pred, P_pred, measurement, measurement_fcn, R, alpha, kappa, beta)
    nx = length(x_pred);
    ny = length(measurement);

    lambda = alpha^2*(nx+kappa)-nx;
    Wm = [lambda/(nx+lambda); 0.5/(nx+lambda)*ones(2*nx,1)];
    Wc = Wm; Wc(1) = Wc(1) + (1-alpha^2+beta);

    % Sigma Points
    sqrtP = chol((nx+lambda)*P_pred,'lower');
    X_sigma = [x_pred, x_pred+sqrtP, x_pred-sqrtP];

    % Propagasi ke ruang pengukuran
    Y_sigma = zeros(ny,2*nx+1);
    for i=1:2*nx+1
        Y_sigma(:,i) = measurement_fcn(X_sigma(:,i));
    end

    % Mean measurement
    y_hat = Y_sigma*Wm;

    % Covariances
    P_yy = R; P_xy = zeros(nx,ny);
    for i=1:2*nx+1
        dy = Y_sigma(:,i)-y_hat;
        dx = X_sigma(:,i)-x_pred;
        P_yy = P_yy + Wc(i)*(dy*dy');
        P_xy = P_xy + Wc(i)*(dx*dy');
    end

    % Gain dan update
    K = P_xy/P_yy;
    x_est = x_pred + K*(measurement-y_hat);
    P = P_pred - K*P_yy*K';
end
