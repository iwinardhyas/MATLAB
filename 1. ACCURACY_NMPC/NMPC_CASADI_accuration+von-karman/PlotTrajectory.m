% Definisikan vektor waktu yang sesuai dengan data simulasi
time_vector = 0:dt:T_sim; % Ini akan memiliki N_sim + 1 titik

% Plot the states.
fig = figure(1);       % Fokus pada figure 1
set(fig, 'Name', 'Drone Position Plot', 'NumberTitle', 'off'); % Beri nama pada window
clf(fig); % Bersihkan figure sebelum plotting baru

% Plot Posisi X
subplot(2,3,1); % Menggunakan 3 baris, 1 kolom, plot pertama
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
subplot(2,3,2); % Plot kedua
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
subplot(2,3,3); % Plot ketiga
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
subplot(2,3,4); % Plot ketiga
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
subplot(2,3,5); % Plot ketiga
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
subplot(2,3,6); % Plot ketiga
hold on;
plot(time_vector, history_x(6,:), 'b-', 'LineWidth', 1.5); % Actual Z (baris 3)
plot(time_vector, history_x_ref(6,:), 'r--', 'LineWidth', 1.5); % Reference Z (baris 3)
grid on;
xlabel('Time (s)');
ylabel('Position psi (m)');
legend('Actual psi angle', 'Reference psi angle', 'Location','best');
title('Quadrotor psi angle');
hold off;

% Plot thrust
t = 0:dt:T_sim-dt;   % sumbu waktu
figure;
grid on;
plot(t, history_u');
xlabel('Time [s]');
ylabel('Control Input');
legend('u1','u2','u3','u4');   % sesuai nu
title('History of Control Inputs');
hold off;

% du = diff(history_u,1,2);                % ukuran = nu x (N_sim-1)
% t_du = 0:dt:(size(du,2)-1)*dt;           % panjang sama persis
% figure;
% plot(t_du, du');
% xlabel('Time [s]');
% ylabel('\Delta u');
% legend('du1','du2','du3','du4');
% title('Control Smoothness (Delta U) vs Time');
% grid on;




%% Plotting Hasil Lintasan

% Pastikan Anda memiliki data history dari simulasi Anda:
% history_x_ref: Matriks (12 x N_sim_total+1) yang menyimpan state referensi di setiap waktu
% history_x_actual: Matriks (12 x N_sim_total+1) yang menyimpan state aktual di setiap waktu

% Jika T_sim Anda adalah sebuah vektor waktu yang sesuai dengan kolom history_x_ref/actual
% Misalnya: T_sim = 0:dt_ukf:N_sim_total*dt_ukf;

% Ekstrak posisi (x,y,z) dari data history
x_ref = history_x_ref(1, :);
y_ref = history_x_ref(2, :);
z_ref = history_x_ref(3, :);

x_actual = history_x(1, :);
y_actual = history_x(2, :);
z_actual = history_x(3, :);

% Buat figure baru untuk plot 3D
figure;
hold on; % Menahan plot agar bisa menambahkan beberapa garis

% Plot Lintasan Referensi
plot3(x_ref, y_ref, z_ref, 'b--', 'LineWidth', 1.5, 'DisplayName', 'Lintasan Referensi');

% Plot Lintasan Aktual
plot3(x_actual, y_actual, z_actual, 'r-', 'LineWidth', 1.5, 'DisplayName', 'Lintasan Aktual (NMPC)');

% Tandai titik awal dan akhir
plot3(x_ref(1), y_ref(1), z_ref(1), 'bo', 'MarkerFaceColor', 'b', 'MarkerSize', 8, 'DisplayName', 'Awal Referensi');
plot3(x_actual(1), y_actual(1), z_actual(1), 'ro', 'MarkerFaceColor', 'r', 'MarkerSize', 8, 'DisplayName', 'Awal Aktual');

plot3(x_ref(end), y_ref(end), z_ref(end), 'bx', 'MarkerSize', 10, 'LineWidth', 2, 'DisplayName', 'Akhir Referensi');
plot3(x_actual(end), y_actual(end), z_actual(end), 'rx', 'MarkerSize', 10, 'LineWidth', 2, 'DisplayName', 'Akhir Aktual');


% Tambahkan label dan judul
xlabel('Posisi X (m)');
ylabel('Posisi Y (m)');
zlabel('Posisi Z (m)');
title('Lintasan Quadrotor: Referensi vs. Aktual');
grid on;
legend('Location', 'best'); % Menampilkan legenda
axis equal; % Penting untuk memastikan skala sumbu sama, agar lintasan terlihat proporsional
view(45, 30); % Mengatur sudut pandang (azimuth, elevation) untuk tampilan 3D yang lebih baik
hold off;

% figure;
% grid on;
% drone_Animation(time_vector,x_actual,y_actual,z_actual,history_x(4, :),history_x(5, :),history_x(6, :))
% grid off;

figure('Name', 'Von Karman Turbulence Wind Simulation');

% Plot Wind Components
subplot(3,1,1);
plot(time_wind, ug, 'b');
title('Longitudinal Wind Component (ug)');
ylabel('Velocity (m/s)');
grid on;

subplot(3,1,2);
plot(time_wind, vg, 'r');
title('Lateral Wind Component (vg)');
ylabel('Velocity (m/s)');
grid on;

subplot(3,1,3);
plot(time_wind, wg, 'g');
title('Vertical Wind Component (wg)');
ylabel('Velocity (m/s)');
xlabel('Time (seconds)');
grid on;

% Plot Wind Magnitude
figure('Name', 'Total Wind Velocity Magnitude');
plot(time_wind, wind_magnitude);
title('Total Wind Velocity Magnitude');
ylabel('Velocity (m/s)');
xlabel('Time (seconds)');
grid on;

% Plot Wind Direction (Horizontal)
% Wind direction relative to the X (ug) and Y (vg) axes
wind_direction_deg = atan2(vg, ug) * 180 / pi;

figure('Name', 'Horizontal Wind Direction');
plot(time_wind, wind_direction_deg);
title('Wind Direction (Degrees)');
ylabel('Direction (Degrees)');
xlabel('Time (seconds)');
grid on;

%% Plot ug and vg Diagnostics

figure('Name', 'Wind Components ug and vg Diagnostics');

% Plot Longitudinal Wind Component (ug)
subplot(2,1,1);
plot(time_wind, ug, 'b');
hold on;
% Add line for Mean Wind Speed (V_w)
plot(time_wind, V_w * ones(size(time_wind)), 'r--', 'LineWidth', 2);
hold off;
title('Longitudinal Wind Component (ug) vs Time');
ylabel('ug (m/s)');
xlabel('Time (seconds)');
legend('Simulated ug', ['Mean Wind (V_w = ', num2str(V_w), ' m/s)'], 'Location', 'best');
grid on;

% Plot Lateral Wind Component (vg)
subplot(2,1,2);
plot(time_wind, vg, 'b');
hold on;
% Add lines for standard deviation (sigma_v)
plot(time_wind, zeros(size(time_wind)), 'k--', 'LineWidth', 1); % Zero line
plot(time_wind, sigma_v * ones(size(time_wind)), 'r--', 'LineWidth', 2); % +Sigma_v
plot(time_wind, -sigma_v * ones(size(time_wind)), 'r--', 'LineWidth', 2); % -Sigma_v
hold off;
title('Lateral Wind Component (vg) vs Time');
ylabel('vg (m/s)');
xlabel('Time (seconds)');
legend('Simulated vg', 'Mean', ['+/- Sigma_v (', num2str(sigma_v), ' m/s)'], 'Location', 'best');
grid on;