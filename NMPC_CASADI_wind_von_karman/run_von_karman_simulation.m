% run_von_karman_simulation.m
% Skrip untuk menjalankan simulasi angin Von Karman dan plot hasilnya

clear all;
close all;
clc;

%% Pengaturan Parameter Angin (Contoh Turbulensi Sedang)

% Kecepatan rata-rata angin drone (m/s). Jika drone bergerak, ini adalah True Airspeed.
V_w = 15; 

% Intensitas Turbulensi (Standard Deviation)
sigma_u = 1.5; % Longitudinal (m/s)
sigma_v = 1.0; % Lateral (m/s)
sigma_w = 0.8; % Vertikal (m/s)

% Skala Panjang Turbulensi (Scale Lengths)
% Umumnya Lw=0.5*Lu, Lv=Lu
Lu = 300; % (m)
Lv = Lu;  % (m)
Lw = Lu / 2; % (m)

% Pengaturan Simulasi
Ts = 0.05;   % Periode sampling (detik)
T_sim = 8;%120; % Total waktu simulasi (detik)

%% Jalankan Simulasi Angin Von Karman
[time, ug, vg, wg, wind_magnitude] = generate_von_karman_wind(V_w, sigma_u, sigma_v, sigma_w, Lu, Lv, Lw, Ts, T_sim);

%% Plot Hasil

figure('Name', 'Simulasi Angin Turbulensi Von Karman (Q1 Standard)');

% Plot Komponen Angin
subplot(3,1,1);
plot(time, ug, 'b');
title('Komponen Angin Longitudinal (ug)');
ylabel('Kecepatan (m/s)');
grid on;

subplot(3,1,2);
plot(time, vg, 'r');
title('Komponen Angin Lateral (vg)');
ylabel('Kecepatan (m/s)');
grid on;

subplot(3,1,3);
plot(time, wg, 'g');
title('Komponen Angin Vertikal (wg)');
ylabel('Kecepatan (m/s)');
xlabel('Waktu (detik)');
grid on;

% Plot Magnitudo Angin
figure('Name', 'Magnitudo Kecepatan Angin Total');
plot(time, wind_magnitude);
title('Magnitudo Kecepatan Angin Total');
ylabel('Kecepatan (m/s)');
xlabel('Waktu (detik)');
grid on;

% Plot Arah Angin (Horizontal)
% Arah angin relatif terhadap sumbu X (ug) dan Y (vg)
% Plot Arah Angin (Horizontal)
wind_direction_deg = atan2(vg, ug) * 180 / pi;
% wind_direction_deg = mod(atan2d(vg, ug), 360);
% wind_direction_deg_smoothed = smoothdata(wind_direction_deg, 'movmean', 100);


figure('Name', 'Arah Angin Horizontal');
plot(time, wind_direction_deg);
title('Arah Angin (Derajat)');
ylabel('Arah (Derajat)');
xlabel('Waktu (detik)');
grid on;

%% Plot Diagnostik ug dan vg

figure('Name', 'Diagnostik Komponen Angin ug dan vg');

% Plot Komponen Angin Longitudinal (ug)
subplot(2,1,1);
plot(time, ug, 'b');
hold on;
% Tambahkan garis untuk Kecepatan Angin Rata-rata (V_w)
plot(time, V_w * ones(size(time)), 'r--', 'LineWidth', 2); 
hold off;
title('Komponen Angin Longitudinal (ug) vs Waktu');
ylabel('ug (m/s)');
xlabel('Waktu (detik)');
legend('ug Simulasi', ['Mean Wind (V_w = ', num2str(V_w), ' m/s)'], 'Location', 'best');
grid on;


% Plot Komponen Angin Lateral (vg)
subplot(2,1,2);
plot(time, vg, 'b');
hold on;
% Tambahkan garis untuk standar deviasi (sigma_v)
plot(time, zeros(size(time)), 'k--', 'LineWidth', 1); % Garis nol
plot(time, sigma_v * ones(size(time)), 'r--', 'LineWidth', 2); % +Sigma_v
plot(time, -sigma_v * ones(size(time)), 'r--', 'LineWidth', 2); % -Sigma_v
hold off;
title('Komponen Angin Lateral (vg) vs Waktu');
ylabel('vg (m/s)');
xlabel('Waktu (detik)');
legend('vg Simulasi', 'Mean', ['+/- Sigma_v (', num2str(sigma_v), ' m/s)'], 'Location', 'best');
grid on;