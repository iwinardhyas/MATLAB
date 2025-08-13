clear; clc;

%% --- Parameter ---
nx = 12;      % State dimension
nz = 6;       % Measurement dimension
dt = 0.01;    % Sampling time
Nsim = 500;   % Lama simulasi

%% --- Ground Truth (gerak sederhana) ---
true_state = zeros(nx, Nsim);
for k = 1:Nsim
    t_now = (k-1)*dt;
    xdesired = QuadrotorReferenceTrajectory1(t_now);

    % Misalnya QuadrotorReferenceTrajectory1 mengembalikan [x; y; z; phi; theta; psi; ...]
    % Pastikan bentuknya nx x 1
    true_state(:,k) = xdesired;
end

% Measurement dengan noise
measurements = true_state(1:6,:) + 0.05*randn(6,Nsim);

%% --- Inisialisasi RAUKF ---
x_est = zeros(nx,1);    % Estimasi awal
P = eye(nx);            % Kovariansi state awal
Q = 0.05*eye(nx);       % Kovariansi proses awal
R = 0.05*eye(nz);       % Kovariansi pengukuran awal

% Parameter RAUKF
lambda0 = 0.2; delta0 = 0.2;
a = 5; b = 5;
chi2_thr = chi2inv(0.80, nz);

% Penyimpanan hasil
x_store = zeros(nx,Nsim);
Q_hist = zeros(nx,nx,Nsim);
R_hist = zeros(nz,nz,Nsim);
innovation_hist = zeros(nz,Nsim);

%% --- Loop RAUKF ---
for k = 1:Nsim
    % 1. Generate sigma points
    [Xsig, W] = sigma_points(x_est, P);
    
    % 2. Prediksi state
    X_pred = zeros(nx,size(Xsig,2));
    for i = 1:size(Xsig,2)
        X_pred(:,i) = quadrotor_dynamics(Xsig(:,i), dt);
    end
    x_pred = X_pred * W';
    P_pred = Q;
    for i = 1:size(Xsig,2)
        diff = X_pred(:,i)-x_pred;
        P_pred = P_pred + W(i)*(diff*diff');
    end
    
    % 3. Prediksi measurement
    Zsig = zeros(nz,size(Xsig,2));
    for i = 1:size(Xsig,2)
        Zsig(:,i) = measurement_model(X_pred(:,i));
    end
    z_pred = Zsig * W';
    Pzz = R;
    Pxz = zeros(nx,nz);
    for i = 1:size(Xsig,2)
        dz = Zsig(:,i)-z_pred;
        dx = X_pred(:,i)-x_pred;
        Pzz = Pzz + W(i)*(dz*dz');
        Pxz = Pxz + W(i)*(dx*dz');
    end
    
    % 4. Update state
    K = Pxz / Pzz;
    z_meas = measurements(:,k);
    innov = z_meas - z_pred;
    x_upd = x_pred + K*innov;
    P_upd = P_pred - K*Pzz*K';
    
   % Simpan inovasi
    innovation_hist(:,k) = innov;
    
    % 5. Fault detection (Adaptive Q & R)
    phi = innov'*(Pzz\innov);
    if phi > chi2_thr
        % Adaptive Q
        lambda = max(lambda0, (phi - a*chi2_thr)/phi);
        Q = (1-lambda)*Q + lambda*(K*innov*innov'*K');
        
        % Adaptive R
        delta = max(delta0, (phi - b*chi2_thr)/phi);
        residual = z_meas - measurement_model(x_upd);
        Szz_hat = cov(Zsig'); % aproksimasi Szz
        R = (1-delta)*R + delta*(residual*residual' + Szz_hat);
        
        % Koreksi estimasi
        Pxx_corr = P_upd + Q;
        Pxz_corr = Pxz;
        Pzz_corr = Szz_hat + R;
        K_hat = Pxz_corr / Pzz_corr;
        x_upd = x_upd + K_hat*(z_meas - z_pred);
        P_upd = Pxx_corr - K_hat*Pxz_corr*K_hat';
    end
    
    % Simpan hasil
    x_est = x_upd;
    P = P_upd;
    x_store(:,k) = x_est;
    Q_hist(:,:,k) = Q;
    R_hist(:,:,k) = R;
    disp(Q_hist);
    disp(R_hist);
end
% disp(size(P'));
% disp(Q_hist);
% disp(R_hist);

%% --- Evaluasi ---
time = (0:Nsim-1)*dt;
pos_error = sqrt(mean((x_store(1:3,:) - true_state(1:3,:)).^2, 2));
disp('RMSE posisi (x,y,z):');
disp(pos_error);

%% --- Plot Estimasi vs Ground Truth ---
figure;
subplot(3,1,1);
plot(time,true_state(1,:), 'k-', 'LineWidth',1.5); hold on;
plot(time,x_store(1,:), 'r--','LineWidth',1.5);
xlabel('Time [s]'); ylabel('X [m]');
legend('True','Estimated'); grid on;

subplot(3,1,2);
plot(time,true_state(2,:), 'k-', 'LineWidth',1.5); hold on;
plot(time,x_store(2,:), 'r--','LineWidth',1.5);
xlabel('Time [s]'); ylabel('Y [m]');
legend('True','Estimated'); grid on;

subplot(3,1,3);
plot(time,true_state(3,:), 'k-', 'LineWidth',1.5); hold on;
plot(time,x_store(3,:), 'r--','LineWidth',1.5);
xlabel('Time [s]'); ylabel('Z [m]');
legend('True','Estimated'); grid on;

%% --- Plot Adaptasi Q & R ---
figure;
plot(squeeze(Q_hist(1,1,:)),'b','LineWidth',1.2); hold on;
plot(squeeze(R_hist(1,1,:)),'r','LineWidth',1.2);
xlabel('Time step'); ylabel('Covariance');
legend('Q(1,1)','R(1,1)');
title('Evolusi Kovariansi Adaptif'); grid on;

%% --- Plot Inovasi (Residual Measurement) ---
figure;
for i = 1:nz
    subplot(nz,1,i);
    plot(time, innovation_hist(i,:), 'LineWidth', 1.2);
    ylabel(['Innov ' num2str(i)]);
    grid on;
end
xlabel('Time [s]');
sgtitle('Inovasi (Residual Measurement)');

%% --- Plot 3D Lintasan ---
figure;
plot3(true_state(1,:), true_state(2,:), true_state(3,:), 'k-', 'LineWidth', 1.5); hold on;
plot3(x_store(1,:), x_store(2,:), x_store(3,:), 'r--', 'LineWidth', 1.5);
xlabel('X [m]'); ylabel('Y [m]'); zlabel('Z [m]');
legend('True Trajectory','Estimated Trajectory');
grid on; axis equal;
title('3D Trajectory Comparison');
view(45,25);

%% --- Plot Error Posisi per Sumbu ---
pos_error_xyz = x_store(1:3,:) - true_state(1:3,:);
figure;
subplot(3,1,1);
plot(time, pos_error_xyz(1,:), 'b', 'LineWidth', 1.2);
ylabel('Error X [m]'); grid on;

subplot(3,1,2);
plot(time, pos_error_xyz(2,:), 'r', 'LineWidth', 1.2);
ylabel('Error Y [m]'); grid on;

subplot(3,1,3);
plot(time, pos_error_xyz(3,:), 'g', 'LineWidth', 1.2);
ylabel('Error Z [m]'); xlabel('Time [s]'); grid on;
sgtitle('Position Error over Time');

%% --- Plot Orientasi (Roll, Pitch, Yaw) ---
figure;
angles_deg = rad2deg(true_state(4:6,:)); % asumsikan phi, theta, psi di state 4-6
angles_est = rad2deg(x_store(4:6,:));
subplot(3,1,1);
plot(time, angles_deg(1,:), 'k-', time, angles_est(1,:), 'r--', 'LineWidth', 1.2);
ylabel('Roll [deg]'); legend('True','Estimated'); grid on;

subplot(3,1,2);
plot(time, angles_deg(2,:), 'k-', time, angles_est(2,:), 'r--', 'LineWidth', 1.2);
ylabel('Pitch [deg]'); legend('True','Estimated'); grid on;

subplot(3,1,3);
plot(time, angles_deg(3,:), 'k-', time, angles_est(3,:), 'r--', 'LineWidth', 1.2);
ylabel('Yaw [deg]'); xlabel('Time [s]'); legend('True','Estimated'); grid on;
sgtitle('Orientation Comparison');

%% --- Plot RMSE Kumulatif ---
rmse_cum = sqrt(mean((x_store(1:3,:) - true_state(1:3,:)).^2, 2));
figure;
bar(rmse_cum);
set(gca, 'XTickLabel', {'X','Y','Z'});
ylabel('RMSE [m]');
title('Cumulative RMSE (Position)');



function [X, W] = sigma_points(x, P)
    nx = length(x);
    lambda = 1; 
    S = chol(P,'lower');
    X = [x, x + sqrt(nx+lambda)*S, x - sqrt(nx+lambda)*S];
    W0 = lambda/(nx+lambda);
    Wi = 1/(2*(nx+lambda));
    W = [W0, Wi*ones(1,2*nx)];  % 1x(2*nx+1)
end


function x_next = quadrotor_dynamics(x, dt)
    % State: [x y z phi theta psi u v w p q r]
    g = 9.81;
    m = 1.0;
    Ix = 0.02; Iy = 0.02; Iz = 0.04;

    % Ekstraksi state
    phi = x(4); theta = x(5); psi = x(6);
    u = x(7); v = x(8); w = x(9);
    p = x(10); q = x(11); r = x(12);

    % Translasi
    xdot = u;
    ydot = v;
    zdot = w;

    % Euler angle rates
    phidot = p + q*sin(phi)*tan(theta) + r*cos(phi)*tan(theta);
    thetadot = q*cos(phi) - r*sin(phi);
    psidot = q*sin(phi)/cos(theta) + r*cos(phi)/cos(theta);

    % Linear acceleration (asumsi thrust = mg)
    udot = r*v - q*w - g*sin(theta);
    vdot = p*w - r*u + g*sin(phi)*cos(theta);
    wdot = q*u - p*v + g*cos(phi)*cos(theta) - g;

    % Angular acceleration (asumsi kontrol sederhana)
    pdot = 0; qdot = 0; rdot = 0;

    % Integrasi Euler maju
    x_next = x + dt * [xdot; ydot; zdot; phidot; thetadot; psidot; udot; vdot; wdot; pdot; qdot; rdot];
end

function z = measurement_model(x)
    % Sensor membaca posisi & orientasi
    z = x(1:6);
end

function z = get_measurement(k)
    % Misal sensor = posisi & orientasi dengan noise
    true_state = zeros(12,1); % ganti dengan model ground-truth
    z = true_state(1:6) + 0.05*randn(6,1);
end
