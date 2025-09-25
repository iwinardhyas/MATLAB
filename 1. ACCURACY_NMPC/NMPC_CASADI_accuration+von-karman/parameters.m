%% 2. Definisikan Parameter Sistem
% Sama seperti yang Anda miliki di model quadrotor Anda
m = 0.5;    % Massa (kg)
g = 9.81;   % Gravitasi (m/s^2)
l = 0.25;   % Panjang lengan (m)
Ixx = 4.85e-3; % Momen inersia
Iyy = 4.85e-3;
Izz = 8.81e-3;

% Dimensi State dan Input
nx = 12; % Posisi (x,y,z), Orientasi (phi,theta,psi), Kecepatan Linear (vx,vy,vz), Kecepatan Angular (p,q,r)
nu = 4;  % Dorong masing-masing motor (f1, f2, f3, f4)

% Horizon Prediksi NMPC
N = 40;%20; % Prediction horizon (langkah waktu)
dt = 0.05; % Ukuran langkah waktu diskretisasi (detik)

% Referensi (Target)
% x_ref = [0;0;1;0;0;0;0;0;0;0;0;0]; % Contoh: target ketinggian 1m, diam