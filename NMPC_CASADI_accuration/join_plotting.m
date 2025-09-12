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
