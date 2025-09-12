% Load hasil simulasi
load('sim_single.mat','results'); results_single = results;
load('sim_multi.mat','results');  results_multi  = results;

% --- Time vector untuk state (N_sim+1) ---
t_single = (0:size(results_single.history_x,2)-1)*results_single.dt;
t_multi  = (0:size(results_multi.history_x,2)-1)*results_multi.dt;

% --- Time vector untuk kontrol (N_sim) ---
t_u_single = (0:size(results_single.history_u,2)-1) * results_single.dt;
t_u_multi  = (0:size(results_multi.history_u,2)-1) * results_multi.dt;

% --- Hitung error tiap state ---
% Posisi
err_x_single = results_single.history_x(1,:) - results_single.history_x_ref(1,:);
err_y_single = results_single.history_x(2,:) - results_single.history_x_ref(2,:);
err_z_single = results_single.history_x(3,:) - results_single.history_x_ref(3,:);

err_x_multi = results_multi.history_x(1,:) - results_multi.history_x_ref(1,:);
err_y_multi = results_multi.history_x(2,:) - results_multi.history_x_ref(2,:);
err_z_multi = results_multi.history_x(3,:) - results_multi.history_x_ref(3,:);

% Orientasi
err_phi_single   = results_single.history_x(4,:) - results_single.history_x_ref(4,:);
err_theta_single = results_single.history_x(5,:) - results_single.history_x_ref(5,:);
err_psi_single   = results_single.history_x(6,:) - results_single.history_x_ref(6,:);

err_phi_multi   = results_multi.history_x(4,:) - results_multi.history_x_ref(4,:);
err_theta_multi = results_multi.history_x(5,:) - results_multi.history_x_ref(5,:);
err_psi_multi   = results_multi.history_x(6,:) - results_multi.history_x_ref(6,:);

% Thrust (sum dari 4 motor, bandingkan dengan rata2 hover thrust)
thrust_single = sum(results_single.history_u,1);
thrust_multi  = sum(results_multi.history_u,1);
thrust_ref = mean(thrust_single); % approx hover thrust
err_thrust_single = thrust_single - thrust_ref;
err_thrust_multi  = thrust_multi - thrust_ref;

<<<<<<< HEAD
% ======================================
% Delta U (smoothness control effort)
% ======================================
deltaU_single = diff(results_single.history_u,1,2); % diff sepanjang waktu
deltaU_multi  = diff(results_multi.history_u,1,2);

norm_deltaU_single = vecnorm(deltaU_single); % norma tiap step
norm_deltaU_multi  = vecnorm(deltaU_multi);

% ======================================
% Error tracking posisi (xyz)
% ======================================
xyz_single = results_single.history_x(1:3,:); % (x,y,z) actual
xyz_multi  = results_multi.history_x(1:3,:);

xyz_ref = results_single.history_ref(1:3,:);  % referensi (asumsi sama)
err_pos_single = vecnorm(xyz_single - xyz_ref);
err_pos_multi  = vecnorm(xyz_multi - xyz_ref);

% ======================================
% Plotting
% ======================================
t_single = results_single.t;
t_multi  = results_multi.t;
=======
% --- Ambil posisi dari hasil simulasi ---
x_single = results_single.history_x(1,:);
y_single = results_single.history_x(2,:);
z_single = results_single.history_x(3,:);

x_multi  = results_multi.history_x(1,:);
y_multi  = results_multi.history_x(2,:);
z_multi  = results_multi.history_x(3,:);

% --- Ambil data kontrol ---
u_single = results_single.history_u;   % [nu x N_sim]
u_multi  = results_multi.history_u;    % [nu x N_sim]

dt_single = results_single.dt;
dt_multi  = results_multi.dt;

% --- Time vectors ---
t_u_single = (0:size(u_single,2)-1) * dt_single;
t_u_multi  = (0:size(u_multi,2)-1) * dt_multi;

% --- Δu ---
du_single = diff(u_single,1,2);
du_multi  = diff(u_multi,1,2);

t_du_single = (0:size(du_single,2)-1) * dt_single;
t_du_multi  = (0:size(du_multi,2)-1) * dt_multi;

% --- Δ²u (jerk) ---
d2u_single = diff(du_single,1,2);
d2u_multi  = diff(du_multi,1,2);

t_d2u_single = (0:size(d2u_single,2)-1) * dt_single;
t_d2u_multi  = (0:size(d2u_multi,2)-1) * dt_multi;

% --- Total Variation (TV) ---
tv_single = sum(abs(du_single),1);
tv_multi  = sum(abs(du_multi),1);

tv_cum_single = cumsum(tv_single);
tv_cum_multi  = cumsum(tv_multi);

% --- Control Energy ---
u_energy_single = sum(u_single.^2,1);
u_energy_multi  = sum(u_multi.^2,1);

% --- FFT (contoh u1 saja) ---
Fs_single = 1/dt_single;
nfft_single = length(u_single(1,:));
f_single = Fs_single*(0:(nfft_single/2))/nfft_single;

U_fft_single = fft(u_single(1,:));
P2_single = abs(U_fft_single/nfft_single);
P1_single = P2_single(1:nfft_single/2+1);
P1_single(2:end-1) = 2*P1_single(2:end-1);

Fs_multi = 1/dt_multi;
nfft_multi = length(u_multi(1,:));
f_multi = Fs_multi*(0:(nfft_multi/2))/nfft_multi;

U_fft_multi = fft(u_multi(1,:));
P2_multi = abs(U_fft_multi/nfft_multi);
P1_multi = P2_multi(1:nfft_multi/2+1);
P1_multi(2:end-1) = 2*P1_multi(2:end-1);

>>>>>>> 26a314139bdbaa8e0a37ca3a17db7b112e733232

% --- Plot ---
figure;

subplot(7,1,1);
plot(t_single,err_x_single,'b','LineWidth',1.2); hold on;
plot(t_multi,err_x_multi,'r--','LineWidth',1.2);
% plot(t_single,results_single.history_x_ref(1,:),'k:','LineWidth',1.5);
ylabel('x err [m]'); legend('Single','Multi');

subplot(7,1,2);
plot(t_single,err_y_single,'b','LineWidth',1.2); hold on;
plot(t_multi,err_y_multi,'r--','LineWidth',1.2);
% plot(t_single,results_single.history_x_ref(2,:),'k:','LineWidth',1.5);
ylabel('y err [m]');

subplot(7,1,3);
plot(t_single,err_z_single,'b','LineWidth',1.2); hold on;
plot(t_multi,err_z_multi,'r--','LineWidth',1.2);
% plot(t_single,results_single.history_x_ref(3,:),'k:','LineWidth',1.5);
ylabel('z err [m]');

subplot(7,1,4);
plot(t_single,err_phi_single,'b','LineWidth',1.2); hold on;
plot(t_multi,err_phi_multi,'r--','LineWidth',1.2);
% plot(t_single,results_single.history_x_ref(4,:),'k:','LineWidth',1.5);
ylabel('\phi err [rad]');

subplot(7,1,5);
plot(t_single,err_theta_single,'b','LineWidth',1.2); hold on;
plot(t_multi,err_theta_multi,'r--','LineWidth',1.2);
% plot(t_single,results_single.history_x_ref(5,:),'k:','LineWidth',1.5);
ylabel('\theta err [rad]');

subplot(7,1,6);
plot(t_single,err_psi_single,'b','LineWidth',1.2); hold on;
plot(t_multi,err_psi_multi,'r--','LineWidth',1.2);
% plot(t_single,results_single.history_x_ref(6,:),'k:','LineWidth',1.5);
ylabel('\psi err [rad]');

subplot(7,1,7);
plot(t_u_single,err_thrust_single,'b','LineWidth',1.2); hold on;
plot(t_u_multi,err_thrust_multi,'r--','LineWidth',1.2);
ylabel('Thrust err'); xlabel('Time [s]');

sgtitle('Tracking Error Comparison (Single vs Multiple Shooting)');

<<<<<<< HEAD
figure;

subplot(3,1,1);
plot(t_single, err_thrust_single, 'b','LineWidth',1.2); hold on;
plot(t_multi, err_thrust_multi, 'r--','LineWidth',1.2);
yline(0,'k--');
xlabel('Time [s]'); ylabel('Thrust Error [N]');
title('Thrust Error vs Hover');
legend('Single penalty','Multi penalty');

subplot(3,1,2);
plot(t_single(2:end), norm_deltaU_single, 'b','LineWidth',1.2); hold on;
plot(t_multi(2:end), norm_deltaU_multi, 'r--','LineWidth',1.2);
xlabel('Time [s]'); ylabel('||ΔU||');
title('Control Smoothness (Delta U)');
legend('Single penalty','Multi penalty');

subplot(3,1,3);
plot(t_single, err_pos_single, 'b','LineWidth',1.2); hold on;
plot(t_multi, err_pos_multi, 'r--','LineWidth',1.2);
xlabel('Time [s]'); ylabel('||pos error|| [m]');
title('Position Tracking Error');
legend('Single penalty','Multi penalty');

sgtitle('Comparison of Penalty Methods in NMPC');
=======
% --- Plot 3D Trajectory ---
figure;
plot3(x_single, y_single, z_single, 'b-', 'LineWidth', 1.5); hold on;
plot3(x_multi,  y_multi,  z_multi,  'r--', 'LineWidth', 1.5);

% Tambahkan marker awal dan akhir
plot3(x_single(1), y_single(1), z_single(1), 'go', 'MarkerSize', 8, 'MarkerFaceColor','g'); % start
plot3(x_single(end), y_single(end), z_single(end), 'ko', 'MarkerSize', 8, 'MarkerFaceColor','k'); % end single
plot3(x_multi(end), y_multi(end), z_multi(end), 'mo', 'MarkerSize', 8, 'MarkerFaceColor','m'); % end multi

grid on; axis equal;
xlabel('X [m]'); ylabel('Y [m]'); zlabel('Z [m]');
title('3D Trajectory Comparison: Single Shooting vs Multiple Shooting NMPC');
legend('Single Shooting','Multiple Shooting','Start','End (Single)','End (Multi)');
view(45,25); % atur sudut pandang 3D

% ---- Plot all in one figure ----
figure;

subplot(3,2,1);
plot(t_u_single, u_single', 'b'); hold on;
plot(t_u_multi, u_multi', 'r--'); grid on;
xlabel('Time [s]'); ylabel('u');
title('Control Inputs');
legend('u1-single','u2-single','u3-single','u4-single',...
       'u1-multi','u2-multi','u3-multi','u4-multi');

subplot(3,2,2);
plot(t_du_single, du_single','b'); hold on;
plot(t_du_multi, du_multi','r--'); grid on;
xlabel('Time [s]'); ylabel('\Delta u');
title('Control Smoothness (\Delta u)');

subplot(3,2,3);
plot(t_du_single, tv_cum_single,'b','LineWidth',1.5); hold on;
plot(t_du_multi, tv_cum_multi,'r--','LineWidth',1.5); grid on;
xlabel('Time [s]'); ylabel('Cumulative |Δu|');
title('Cumulative Control Variation (TV)');
legend('Single','Multi');

subplot(3,2,4);
plot(t_u_single, u_energy_single,'b','LineWidth',1.5); hold on;
plot(t_u_multi, u_energy_multi,'r--','LineWidth',1.5); grid on;
xlabel('Time [s]'); ylabel('\Sigma u^2');
title('Instantaneous Control Energy');
legend('Single','Multi');

subplot(3,2,5);
plot(t_d2u_single, d2u_single','b'); hold on;
plot(t_d2u_multi, d2u_multi','r--'); grid on;
xlabel('Time [s]'); ylabel('\Delta^2 u');
title('Jerk (Second Difference of Control)');

subplot(3,2,6);
plot(f_single, P1_single,'b','LineWidth',1.5); hold on;
plot(f_multi,  P1_multi, 'r--','LineWidth',1.5); grid on;
xlabel('Frequency [Hz]'); ylabel('|U1(f)|');
title('Frequency Spectrum of u1');
legend('Single','Multi');

sgtitle('Comprehensive Control Analysis (u, Δu, TV, Energy, Jerk, FFT)');
>>>>>>> 26a314139bdbaa8e0a37ca3a17db7b112e733232
