% Definisikan vektor waktu yang sesuai dengan data simulasi
time_vector = 0:dt:T_sim; % Ini akan memiliki N_sim + 1 titik

% Plot the states.
fig = figure(1);       % Fokus pada figure 1
set(fig, 'Name', 'Drone Position Plot', 'NumberTitle', 'off'); % Beri nama pada window
clf(fig); % Bersihkan figure sebelum plotting baru

% Plot Posisi X
subplot(3,4,1); % Menggunakan 3 baris, 1 kolom, plot pertama
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
subplot(3,4,2); % Plot kedua
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
subplot(3,4,3); % Plot ketiga
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
subplot(3,4,4); % Plot ketiga
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
subplot(3,4,5); % Plot ketiga
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
subplot(3,4,6); % Plot ketiga
hold on;
plot(time_vector, history_x(6,:), 'b-', 'LineWidth', 1.5); % Actual Z (baris 3)
plot(time_vector, history_x_ref(6,:), 'r--', 'LineWidth', 1.5); % Reference Z (baris 3)
grid on;
xlabel('Time (s)');
ylabel('Position psi (m)');
legend('Actual psi angle', 'Reference psi angle', 'Location','best');
title('Quadrotor psi angle');
hold off;

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