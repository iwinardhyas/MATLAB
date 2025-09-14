% Definisikan vektor waktu yang sesuai dengan data simulasi
time_vector = 0:dt:T_sim; % Ini akan memiliki N_sim + 1 titik

% % Plot the states.
% fig = figure(1);       % Fokus pada figure 1
% set(fig, 'Name', 'Drone Position Plot', 'NumberTitle', 'off'); % Beri nama pada window
% clf(fig); % Bersihkan figure sebelum plotting baru
% 
% % Plot Posisi X
% subplot(3,3,1); % Menggunakan 3 baris, 1 kolom, plot pertama
% hold on;
% plot(time_vector, history_x(1,:), 'b-', 'LineWidth', 1.5); % Actual X (baris 1)
% plot(time_vector, history_x_ref(1,:), 'r--', 'LineWidth', 1.5); % Reference X (baris 1)
% grid on;
% xlabel('Time (s)');
% ylabel('Position X (m)');
% legend('Actual X', 'Reference X', 'Location','best');
% title('Quadrotor X Position');
% hold off;
% 
% % Plot Posisi Y
% subplot(3,3,2); % Plot kedua
% hold on;
% plot(time_vector, history_x(2,:), 'b-', 'LineWidth', 1.5); % Actual Y (baris 2)
% plot(time_vector, history_x_ref(2,:), 'r--', 'LineWidth', 1.5); % Reference Y (baris 2)
% grid on;
% xlabel('Time (s)');
% ylabel('Position Y (m)');
% legend('Actual Y', 'Reference Y', 'Location','best');
% title('Quadrotor Y Position');
% hold off;
% 
% % Plot Posisi Z
% subplot(3,3,3); % Plot ketiga
% hold on;
% plot(time_vector, history_x(3,:), 'b-', 'LineWidth', 1.5); % Actual Z (baris 3)
% plot(time_vector, history_x_ref(3,:), 'r--', 'LineWidth', 1.5); % Reference Z (baris 3)
% grid on;
% xlabel('Time (s)');
% ylabel('Position Z (m)');
% legend('Actual Z', 'Reference Z', 'Location','best');
% title('Quadrotor Z Position');
% hold off;
% 
% % Plot Posisi phi angle
% subplot(3,3,4); % Plot ketiga
% hold on;
% plot(time_vector, history_x(4,:), 'b-', 'LineWidth', 1.5); % Actual Z (baris 3)
% plot(time_vector, history_x_ref(4,:), 'r--', 'LineWidth', 1.5); % Reference Z (baris 3)
% grid on;
% xlabel('Time (s)');
% ylabel('Position phi (m)');
% legend('Actual phi angle', 'Reference phi angle', 'Location','best');
% title('Quadrotor phi angle');
% hold off;
% 
% % Plot Posisi theta angle
% subplot(3,3,5); % Plot ketiga
% hold on;
% plot(time_vector, history_x(5,:), 'b-', 'LineWidth', 1.5); % Actual Z (baris 3)
% plot(time_vector, history_x_ref(5,:), 'r--', 'LineWidth', 1.5); % Reference Z (baris 3)
% grid on;
% xlabel('Time (s)');
% ylabel('Position theta (m)');
% legend('Actual theta angle', 'Reference theta angle', 'Location','best');
% title('Quadrotor theta angle');
% hold off;
% 
% % Plot Posisi psi angle
% subplot(3,3,6); % Plot ketiga
% hold on;
% plot(time_vector, history_x(6,:), 'b-', 'LineWidth', 1.5); % Actual Z (baris 3)
% plot(time_vector, history_x_ref(6,:), 'r--', 'LineWidth', 1.5); % Reference Z (baris 3)
% grid on;
% xlabel('Time (s)');
% ylabel('Position psi (m)');
% legend('Actual psi angle', 'Reference psi angle', 'Location','best');
% title('Quadrotor psi angle');
% hold off;
% 
% % Plot thrust
% t = 0:dt:T_sim-dt;   % sumbu waktu
% figure;
% grid on;
% plot(t, history_u');
% xlabel('Time [s]');
% ylabel('Control Input');
% legend('u1','u2','u3','u4');   % sesuai nu
% title('History of Control Inputs');
% hold off;
% 
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

% Plot Rintangan (Obstacle)
num_obs = size(obs_center, 2); % Mendapatkan jumlah rintangan
for j = 1:num_obs
    % Ambil koordinat pusat dan jari-jari untuk rintangan saat ini
    c = obs_center(:, j);
    r = obs_radius(j);
    obstacle_height = 8; % Tentukan tinggi silinder Anda

    % Membuat data silinder (cylinder) dengan radius r dan tinggi 1
    [x_cyl, y_cyl, z_cyl] = cylinder(r);
    
    % Mengatur tinggi silinder
    z_cyl = z_cyl * obstacle_height;
    
    % Menggeser posisi silinder ke koordinat pusat rintangan (c)
    x_cyl = x_cyl + c(1);
    y_cyl = y_cyl + c(2);
%     z_cyl = z_cyl + c(3) - (obstacle_height / 2); % Menggeser z agar berpusat di titik c(3)
    
    % Menampilkan plot silinder
    surf(x_cyl, y_cyl, z_cyl, 'FaceAlpha', 0.5, 'EdgeColor', 'none', 'DisplayName', ['Obstacle ' num2str(j)]);
end

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

figure; hold on; grid on;
plot3(history_x(1,:), history_x(2,:), history_x(3,:), 'b-', 'LineWidth', 1.5); % jalur drone
plot3(history_x_ref(1,:), history_x_ref(2,:), history_x_ref(3,:), 'r--');     % jalur referensi asli

% plot obstacle
[xx,yy,zz] = sphere(20);
for j=1:size(obs_center,2)
    surf(obs_center(1,j) + obs_radius(j)*xx, ...
         obs_center(2,j) + obs_radius(j)*yy, ...
         obs_center(3,j) + obs_radius(j)*zz, ...
         'FaceAlpha',0.3,'EdgeColor','none','FaceColor','g');
end

xlabel('X'); ylabel('Y'); zlabel('Z');
legend('Drone path','Reference path','Obstacles');
title('Trajektori Drone dengan APF + NMPC');

d_min_log = zeros(1,N_sim);
for i=1:N_sim
    p = history_x(1:3,i);
    d_all = vecnorm(p - obs_center,2,1) - obs_radius;
    d_min_log(i) = min(d_all);
end

figure; plot((0:N_sim-1)*dt, d_min_log, 'LineWidth',1.5);
xlabel('Time [s]'); ylabel('Min distance to obstacle [m]');
title('Jarak Minimum Drone ke Obstacle');
grid on;

% Tambahkan baris ketiga (z) dengan nilai nol untuk plot 3D
z_ref_history = zeros(1, size(v_ref_history, 2));

figure; plot3(v_ref_history(1,:), v_ref_history(2,:),z_ref_history);
xlabel('v_x'); ylabel('v_y');
title('Arah koreksi dari APF');
grid on;
