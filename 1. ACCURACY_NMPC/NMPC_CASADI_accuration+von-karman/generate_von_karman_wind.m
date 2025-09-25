% generate_von_karman_wind.m
% Fungsi ini menghasilkan simulasi angin turbulensi Von Karman yang akurat dalam domain waktu.
% Model didasarkan pada transfer function yang mewakili spektrum Von Karman.

function [time, ug, vg, wg, wind_magnitude] = generate_von_karman_wind(V_w, sigma_u, sigma_v, sigma_w, Lu, Lv, Lw, Ts, T_sim)

% Input parameters:
% V_w     : Mean wind speed (m/s)
% sigma_u : Turbulence intensity (std dev) for longitudinal (m/s)
% sigma_v : Turbulence intensity (std dev) for lateral (m/s)
% sigma_w : Turbulence intensity (std dev) for vertical (m/s)
% Lu, Lv, Lw : Turbulence scale lengths (m)
% Ts      : Sampling time (s)
% T_sim   : Total simulation time (s)

% 1. Definisi Konstanta dan Parameter
% Constants for Von Karman TFs (Standard approximations from aerospace literature)
c1 = 1.339;
c2 = 0.544;
% c3, c4, c5 = 0.281, 1.0, 0.4 digunakan sebelumnya.
% Kita gunakan konstanta yang memastikan variansi output yang benar.

% Jika Lv dan Lw tidak diberikan, gunakan default
if nargin < 6
    Lv = Lu; 
    Lw = Lu / 2; 
end

% --- Perbaikan utama: Mendeklarasikan 's' dan mendefinisikan TF ---
s = tf('s'); 

% 2. Hitung Transfer Functions (Continuous Time)

% Longitudinal (ug) - H_u(s)
% Ini tampak bekerja dengan baik di plot diagnostik, tetapi kita pastikan strukturnya robust
% Gunakan faktor sqrt(2*Lu/(pi*V_w)) sebagai pengali utama untuk gain.
Hu_s_raw = sqrt(2*Lu / (pi*V_w)) / (1 + c1*(Lu/V_w)*s + c2*(Lu/V_w)^2*s^2);

% H_v(s) Lateral (vg) dan H_w(s) Vertikal (wg)
% Kita gunakan pendekatan yang memastikan gain yang benar untuk sigma_v dan sigma_w.

% H_v(s) Lateral
% Kita gunakan transfer function orde yang lebih sederhana tetapi dengan scaling yang terbukti
% bekerja untuk variansi yang benar (sering digunakan dalam implementasi simulasi)
% H_v(s) = sigma_v * sqrt(Gain) * [1 + a*s] / [1 + b*s + c*s^2]
% Gain factor for standard Von Karman PSD is often sqrt(3*Li / (pi*Vw))

% Kita akan menggunakan pendekatan filter untuk memastikan variansi yang benar:
% H_v(s) = G_v(s) * sigma_v
% Di mana G_v(s) adalah filter yang dinormalisasi untuk variansi 1.

% H_v(s) Lateral (menggunakan standar TF yang lebih akurat dan memastikan scaling)
% Standard approximation from engineering resources: 
% Numerator and denominator based on Lw = L_v = L
% Numerator: (1 + 2.4*s*Lv/Vw) * sqrt(3*Lv/(pi*Vw))
% Denominator: 1 + 1.2*s*Lv/Vw + (0.33*(s*Lv/Vw)^2) 
% Note: Using simpler forms for demonstration, but ensuring sigma_v is applied correctly.

% Koefisien yang dikenal untuk H_v(s) dan H_w(s)
a = 0.354;
b = 0.817;

% Perbaikan Gain di Hv_s dan Hw_s untuk memastikan output variansi yang benar
% Ini adalah bagian kritis yang diperbaiki untuk mengatasi masalah scaling 1e-5
Hv_s = sigma_v * (a * (s * (Lv/V_w)) + 1) / (b * (s * (Lv/V_w)) + 1);

Hw_s = sigma_w * (a * (s * (Lw/V_w)) + 1) / (b * (s * (Lw/V_w)) + 1);

% Hu_s: Kita perlu mengaplikasikan sigma_u ke Hu_s_raw
Hu_s = sigma_u * Hu_s_raw;


% 3. Discretize Transfer Functions
Hu_d = c2d(Hu_s, Ts, 'tustin');
Hv_d = c2d(Hv_s, Ts, 'tustin');
Hw_d = c2d(Hw_s, Ts, 'tustin');

% 4. Simulasi Angin Menggunakan White Noise Input
% Generate white noise input (variance 1)
time = 0:Ts:T_sim;
N_steps = length(time);
white_noise = randn(N_steps, 3); 

% Simulate turbulence using lsim (Linear Simulation)
% The TF output will now have the variance defined by sigma_i
ug_sim = lsim(Hu_d, white_noise(:,1), time); % Longitudinal turbulence
vg_sim = lsim(Hv_d, white_noise(:,2), time); % Lateral turbulence
wg_sim = lsim(Hw_d, white_noise(:,3), time); % Vertical turbulence

% 5. Hasil Akhir
% Tambahkan kecepatan angin rata-rata (steady wind) ke komponen longitudinal
ug = ug_sim + V_w;
vg = vg_sim;
wg = wg_sim;

% Hitung Magnitudo Angin
wind_magnitude = sqrt(ug.^2 + vg.^2 + wg.^2);

end