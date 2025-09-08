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

% Plot thrust
t = 0:dt:T_sim-dt;              % sumbu waktu untuk u
du = diff(history_u,1,2);       % Δu
t_du = 0:dt:(size(du,2)-1)*dt;  % sumbu waktu untuk Δu
d2u = diff(du,1,2);             % Δ²u (jerk)
t_d2u = 0:dt:(size(d2u,2)-1)*dt;

% Total Variation (akumulasi |Δu|)
tv = sum(abs(du),1);
tv_cum = cumsum(tv);

% Control energy
u_energy = sum(history_u.^2,1);

% FFT setup
Fs = 1/dt;
nfft = length(history_u);
f = Fs*(0:(nfft/2))/nfft;
U_fft = fft(history_u(1,:)); % contoh u1 saja
P2 = abs(U_fft/nfft);
P1 = P2(1:nfft/2+1);
P1(2:end-1) = 2*P1(2:end-1);

% ---- Plot all in one figure ----
figure;

subplot(3,2,1);
plot(t, history_u'); grid on;
xlabel('Time [s]'); ylabel('u');
title('Control Inputs');
legend('u1','u2','u3','u4');

subplot(3,2,2);
plot(t_du, du'); grid on;
xlabel('Time [s]'); ylabel('\Delta u');
title('Control Smoothness (\Delta u)');
legend('du1','du2','du3','du4');

subplot(3,2,3);
plot(t_du, tv_cum,'LineWidth',1.5); grid on;
xlabel('Time [s]'); ylabel('Cumulative |Δu|');
title('Cumulative Control Variation (TV)');

subplot(3,2,4);
plot(t, u_energy,'LineWidth',1.5); grid on;
xlabel('Time [s]'); ylabel('\Sigma u^2');
title('Instantaneous Control Energy');

subplot(3,2,5);
plot(t_d2u, d2u'); grid on;
xlabel('Time [s]'); ylabel('\Delta^2 u');
title('Jerk (Second Difference of Control)');
legend('d2u1','d2u2','d2u3','d2u4');

subplot(3,2,6);
plot(f, P1,'LineWidth',1.5); grid on;
xlabel('Frequency [Hz]'); ylabel('|U1(f)|');
title('Frequency Spectrum of u1');

sgtitle('Comprehensive Control Analysis (u, Δu, TV, Energy, Jerk, FFT)');





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