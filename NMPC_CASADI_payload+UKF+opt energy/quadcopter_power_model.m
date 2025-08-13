function power_consumption = quadcopter_power_model(state, total_thrust, estimated_power_params_sym, physics_params_sym)
% quadcopter_power_model: Menghitung konsumsi daya instan quadcopter.
% Ini adalah versi yang kompatibel dengan CasADi (symbolic variables).
%
% Input:
%   state:          Vektor state drone [px;py;pz;phi;theta;psi;vx;vy;vz;p;q;r] (12x1).
%                   Digunakan untuk mendapatkan kecepatan linear (vx,vy,vz).
%   total_thrust:   Gaya dorong total yang dihasilkan oleh semua motor (scalar).
%                   (Sum dari U_vars dari NMPC).
%   estimated_power_params_sym: Struct CasADi berisi parameter model daya simbolik.
%                   Contoh: estimated_power_params_sym.P0t, .kT, .kV
%   physics_params_sym: Struct CasADi berisi parameter fisika simbolik (misal, g - gravitasi).
%                       Saat ini, model daya sederhana tidak menggunakan g,
%                       tetapi disertakan untuk konsistensi API.
%
% Output:
%   power_consumption: Daya yang dikonsumsi drone saat ini (Watt, simbolik).

    % <<< PENTING: Pastikan ini ada untuk mengakses fungsi CasADi seperti fmax, power >>>
    import casadi.*

    % Ekstrak parameter daya yang diestimasi (ini adalah simbol CasADi)
    P0t = estimated_power_params_sym.P0t; % Daya nol-thrust (Watt)
    kT  = estimated_power_params_sym.kT;  % Koefisien daya terinduksi
    kV  = estimated_power_params_sym.kV;  % Koefisien daya parasit

    % Ekstrak kecepatan linear dari state
    % state = [px;py;pz;phi;theta;psi;vx;vy;vz;p;q;r]
    vx = state(7);
    vy = state(8);
    vz = state(9);

    % Hitung magnitudo kecepatan drone
    V_drone = sqrt(vx^2 + vy^2 + vz^2);

    % --- Hitung Komponen Daya ---
    
    % 1. Daya Nol-Thrust / Basline Power
    P_baseline = P0t; 

    % 2. Daya Terinduksi (Induced Power)
    % --- Debugging: Periksa dimensi input total_thrust ---
%     fprintf('Debug (quadcopter_power_model): Dimensi total_thrust: [%d, %d]\n', total_thrust.size());

    % Menggunakan CasADi `fmax` untuk memastikan thrust positif
    total_thrust_positive = fmax(0, total_thrust);

    % --- Debugging: Periksa dimensi output fmax ---
%     fprintf('Debug (quadcopter_power_model): Dimensi fmax(0, total_thrust): [%d, %d]\n', total_thrust_positive.size());

    % Gunakan CasADi 'power' fungsi untuk eksponensiasi (Lebih robust untuk simbolik)
    P_induced = kT * power(total_thrust_positive, 1.5); % <<<<<<< PERUBAHAN PENTING DI SINI

    % 3. Daya Parasit (Parasitic Power)
    P_parasitic = kV * V_drone^2;

    % Total konsumsi daya
    power_consumption = P_baseline + P_induced + P_parasitic;
    
    % Pastikan daya tidak negatif (menggunakan fmax untuk CasADi)
    power_consumption = fmax(0, power_consumption); 
end