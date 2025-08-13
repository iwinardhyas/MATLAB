% Definisikan vektor waktu yang sesuai dengan data simulasi
time_vector = 0:dt:T_sim; % Ini akan memiliki N_sim + 1 titik

% Plot the states.
fig = figure(1);       % Fokus pada figure 1
set(fig, 'Name', 'Drone Position Plot', 'NumberTitle', 'off'); % Beri nama pada window
clf(fig); % Bersihkan figure sebelum plotting baru

% Plot Posisi X
subplot(3,3,1); % Menggunakan 3 baris, 1 kolom, plot pertama
hold on;
plot(time_vector, history_x(1,:), 'b-', 'LineWidth', 1.5); % Actual X (baris 1)
plot(time_vector, history_x_ref(1,:), 'r--', 'LineWidth', 1.5); % Reference X (baris 1)
grid on;
xlabel('Time (s)');
ylabel('Position X (m)');
legend('Actual X', 'Reference X', 'Location','best');
title('Quadrotor X Position');
hold off;

% Plot Posisi Y
subplot(3,3,2); % Plot kedua
hold on;
plot(time_vector, history_x(2,:), 'b-', 'LineWidth', 1.5); % Actual Y (baris 2)
plot(time_vector, history_x_ref(2,:), 'r--', 'LineWidth', 1.5); % Reference Y (baris 2)
grid on;
xlabel('Time (s)');
ylabel('Position Y (m)');
legend('Actual Y', 'Reference Y', 'Location','best');
title('Quadrotor Y Position');
hold off;

% Plot Posisi Z
subplot(3,3,3); % Plot ketiga
hold on;
plot(time_vector, history_x(3,:), 'b-', 'LineWidth', 1.5); % Actual Z (baris 3)
plot(time_vector, history_x_ref(3,:), 'r--', 'LineWidth', 1.5); % Reference Z (baris 3)
grid on;
xlabel('Time (s)');
ylabel('Position Z (m)');
legend('Actual Z', 'Reference Z', 'Location','best');
title('Quadrotor Z Position');
hold off;

% Plot Posisi phi angle
subplot(3,3,4); % Plot ketiga
hold on;
plot(time_vector, history_x(4,:), 'b-', 'LineWidth', 1.5); % Actual Z (baris 3)
plot(time_vector, history_x_ref(4,:), 'r--', 'LineWidth', 1.5); % Reference Z (baris 3)
grid on;
xlabel('Time (s)');
ylabel('Position phi (m)');
legend('Actual phi angle', 'Reference phi angle', 'Location','best');
title('Quadrotor phi angle');
hold off;

% Plot Posisi theta angle
subplot(3,3,5); % Plot ketiga
hold on;
plot(time_vector, history_x(5,:), 'b-', 'LineWidth', 1.5); % Actual Z (baris 3)
plot(time_vector, history_x_ref(5,:), 'r--', 'LineWidth', 1.5); % Reference Z (baris 3)
grid on;
xlabel('Time (s)');
ylabel('Position theta (m)');
legend('Actual theta angle', 'Reference theta angle', 'Location','best');
title('Quadrotor theta angle');
hold off;

% Plot Posisi psi angle
subplot(3,3,6); % Plot ketiga
hold on;
plot(time_vector, history_x(6,:), 'b-', 'LineWidth', 1.5); % Actual Z (baris 3)
plot(time_vector, history_x_ref(6,:), 'r--', 'LineWidth', 1.5); % Reference Z (baris 3)
grid on;
xlabel('Time (s)');
ylabel('Position psi (m)');
legend('Actual psi angle', 'Reference psi angle', 'Location','best');
title('Quadrotor psi angle');
hold off;


% % Opsional: Plot 3D Trajectory
% figure(2);
% hold on;
% plot3(history_x(1,:), history_x(2,:), history_x(3,:), 'b-', 'LineWidth', 2); % Actual 3D
% plot3(history_x_ref(1,:), history_x_ref(2,:), history_x_ref(3,:), 'r--', 'LineWidth', 1.5); % Reference 3D
% grid on;
% xlabel('X (m)');
% ylabel('Y (m)');
% zlabel('Z (m)');
% legend('Actual Trajectory', 'Reference Trajectory', 'Location', 'best');
% title('Quadrotor 3D Trajectory');
% view(3); % Set view to 3D
% axis equal; % Penting untuk menjaga proporsi
% hold off;


% figure('Name', 'Quadrotor Position Tracking');
% subplot(3,1,1);
% plot(time_vec, history_x_actual(1,:), 'b', 'LineWidth', 1.5); hold on;
% plot(time_vec, history_ukf_x_est(1,:), 'g--', 'LineWidth', 1.0);
% plot(time_vec, history_x_ref(1,:), 'r:', 'LineWidth', 1.0);
% title('Quadrotor X Position');
% xlabel('Time (s)'); ylabel('Position X (m)');
% legend('Actual X', 'UKF Est X', 'Reference X');
% grid on;
% 
% subplot(3,1,2);
% plot(time_vec, history_x_actual(2,:), 'b', 'LineWidth', 1.5); hold on;
% plot(time_vec, history_ukf_x_est(2,:), 'g--', 'LineWidth', 1.0);
% plot(time_vec, history_x_ref(2,:), 'r:', 'LineWidth', 1.0);
% title('Quadrotor Y Position');
% xlabel('Time (s)'); ylabel('Position Y (m)');
% legend('Actual Y', 'UKF Est Y', 'Reference Y');
% grid on;
% 
% subplot(3,1,3);
% plot(time_vec, history_x_actual(3,:), 'b', 'LineWidth', 1.5); hold on;
% plot(time_vec, history_ukf_x_est(3,:), 'g--', 'LineWidth', 1.0);
% plot(time_vec, history_x_ref(3,:), 'r:', 'LineWidth', 1.0);
% title('Quadrotor Z Position');
% xlabel('Time (s)'); ylabel('Position Z (m)');
% legend('Actual Z', 'UKF Est Z', 'Reference Z');
% grid on;
% 
% figure('Name', 'Quadrotor Velocity Tracking');
% subplot(3,1,1);
% plot(time_vec, history_x_actual(7,:), 'b', 'LineWidth', 1.5); hold on;
% plot(time_vec, history_ukf_x_est(7,:), 'g--', 'LineWidth', 1.0);
% plot(time_vec, history_x_ref(7,:), 'r:', 'LineWidth', 1.0);
% title('Quadrotor X Velocity');
% xlabel('Time (s)'); ylabel('Velocity X (m/s)');
% legend('Actual VX', 'UKF Est VX', 'Reference VX');
% grid on;
% 
% subplot(3,1,2);
% plot(time_vec, history_x_actual(8,:), 'b', 'LineWidth', 1.5); hold on;
% plot(time_vec, history_ukf_x_est(8,:), 'g--', 'LineWidth', 1.0);
% plot(time_vec, history_x_ref(8,:), 'r:', 'LineWidth', 1.0);
% title('Quadrotor Y Velocity');
% xlabel('Time (s)'); ylabel('Velocity Y (m/s)');
% legend('Actual VY', 'UKF Est VY', 'Reference VY');
% grid on;
% 
% subplot(3,1,3);
% plot(time_vec, history_x_actual(9,:), 'b', 'LineWidth', 1.5); hold on;
% plot(time_vec, history_ukf_x_est(9,:), 'g--', 'LineWidth', 1.0);
% plot(time_vec, history_x_ref(9,:), 'r:', 'LineWidth', 1.0);
% title('Quadrotor Z Velocity');
% xlabel('Time (s)'); ylabel('Velocity Z (m/s)');
% legend('Actual VZ', 'UKF Est VZ', 'Reference VZ');
% grid on;
% 
% figure('Name', 'Quadrotor Orientation Tracking');
% subplot(3,1,1);
% plot(time_vec, rad2deg(history_x_actual(4,:)), 'b', 'LineWidth', 1.5); hold on;
% plot(time_vec, rad2deg(history_ukf_x_est(4,:)), 'g--', 'LineWidth', 1.0);
% plot(time_vec, rad2deg(history_x_ref(4,:)), 'r:', 'LineWidth', 1.0);
% title('Quadrotor Roll (Phi)');
% xlabel('Time (s)'); ylabel('Roll (deg)');
% legend('Actual Roll', 'UKF Est Roll', 'Reference Roll');
% grid on;
% 
% subplot(3,1,2);
% plot(time_vec, rad2deg(history_x_actual(5,:)), 'b', 'LineWidth', 1.5); hold on;
% plot(time_vec, rad2deg(history_ukf_x_est(5,:)), 'g--', 'LineWidth', 1.0);
% plot(time_vec, rad2deg(history_x_ref(5,:)), 'r:', 'LineWidth', 1.0);
% title('Quadrotor Pitch (Theta)');
% xlabel('Time (s)'); ylabel('Pitch (deg)');
% legend('Actual Pitch', 'UKF Est Pitch', 'Reference Pitch');
% grid on;
% 
% subplot(3,1,3);
% plot(time_vec, rad2deg(history_x_actual(6,:)), 'b', 'LineWidth', 1.5); hold on;
% plot(time_vec, rad2deg(history_ukf_x_est(6,:)), 'g--', 'LineWidth', 1.0);
% plot(time_vec, rad2deg(history_x_ref(6,:)), 'r:', 'LineWidth', 1.0);
% title('Quadrotor Yaw (Psi)');
% xlabel('Time (s)'); ylabel('Yaw (deg)');
% legend('Actual Yaw', 'UKF Est Yaw', 'Reference Yaw');
% grid on;
% 
% 
% figure('Name', 'Mass Estimation and Control Inputs');
% subplot(2,1,1);
% plot(time_vec, history_actual_mass, 'b', 'LineWidth', 1.5); hold on;
% plot(time_vec, history_ukf_x_est(end-1,:), 'r--', 'LineWidth', 1.0);
% title('Mass Actual vs. UKF Estimated');
% xlabel('Time (s)'); ylabel('Mass (kg)');
% legend('Actual Mass', 'UKF Estimated Mass');
% grid on;
% 
% subplot(2,1,2);
% plot(time_vec(1:end-1), history_u_nmpc(1,:), 'r', 'LineWidth', 1.0); hold on;
% plot(time_vec(1:end-1), history_u_nmpc(2,:), 'g', 'LineWidth', 1.0);
% plot(time_vec(1:end-1), history_u_nmpc(3,:), 'b', 'LineWidth', 1.0);
% plot(time_vec(1:end-1), history_u_nmpc(4,:), 'k', 'LineWidth', 1.0);
% title('NMPC Control Inputs (Motor Thrusts)');
% xlabel('Time (s)'); ylabel('Thrust (N)');
% legend('Motor 1', 'Motor 2', 'Motor 3', 'Motor 4');
% grid on;
% 
% figure;
% subplot(3,1,1); plot(time_vec, history_phi_actual); ylabel('\phi (rad)'); title('Roll Angle');
% subplot(3,1,2); plot(time_vec, history_theta_actual); ylabel('\theta (rad)'); title('Pitch Angle');
% subplot(3,1,3); plot(time_vec, history_psi_actual); ylabel('\psi (rad)'); title('Yaw Angle');
% title('Angle');
% xlabel('Time (s)');
% 
% figure;
% subplot(3,1,1); plot(time_vec, history_p_actual); ylabel('p (rad/s)'); title('Angular Velocity p');
% subplot(3,1,2); plot(time_vec, history_q_actual); ylabel('q (rad/s)'); title('Angular Velocity q');
% subplot(3,1,3); plot(time_vec, history_r_actual); ylabel('r (rad/s)'); title('Angular Velocity r');
% title('Angular velocity');
% xlabel('Time (s)');

% figure;
% plot(time_vec, history_ukf_x_est(end,:), 'r--', 'LineWidth', 2);
% hold on;
% plot(time_vec, history_actual_eta, 'b', 'LineWidth', 1.5);
% legend('Estimated \eta (UKF)', 'Actual \eta');
% ylabel('Motor Efficiency'); xlabel('Time (s)');
% title('Motor Efficiency Estimation vs Actual');