function [F_att, F_rep] = calculate_forces(Pu, Pgoal, Pobs, k_att, k_rep, rho_eff)
% CALCULATE_FORCES Menghitung vektor Gaya Tarik (Attractive) dan Gaya Repulsif (Repulsive)
% Menggunakan CasADi MX untuk integrasi ke dalam kerangka NMPC/Optimasi.

    import casadi.*

    % --- 1. GAYA TARIK (F_att) ---
    % Persamaan: F_att(goal) = -k_att * rho(Pu, Pgoal) * rho_gz
    
    % Vektor dari posisi drone (Pu) ke posisi tujuan (Pgoal)
    r_att_vec = Pgoal - Pu;
    
    % Jarak (rho) antara drone dan tujuan
    rho_att = norm(r_att_vec); 
    
    % Vektor unit (rho_gz) yang mengarah ke tujuan
    % Untuk menghindari pembagian dengan nol (ketika rho_att = 0), kita gunakan if_else CasADi.
    
    % Jika rho_att mendekati nol (drone di tujuan), vektor unit adalah [0;0;0]
    rho_gz_unit = if_else(rho_att > 1e-6, r_att_vec / rho_att, MX.zeros(size(Pu)));
    
    % Hitung gaya tarik
    % Gaya tarik (F_att) sesuai dengan persamaan
    F_att = -k_att * rho_att * rho_gz_unit; 


    % --- 2. GAYA REPULSIF (F_rep) ---
    % Persamaan: 
    % F_rep(obs) = k_rep * [1/rho(Pu, Pobs) - 1/rho_eff] * (1/rho^2) * rho_obs, jika rho <= rho_eff
    % F_rep(obs) = 0, jika rho > rho_eff
    
    % Vektor dari rintangan (Pobs) ke drone (Pu)
    r_rep_vec = Pu - Pobs; 
    
    % Jarak (rho) antara drone dan rintangan
    rho_rep = norm(r_rep_vec);
    
    % Vektor unit (rho_obs) yang mengarah dari rintangan ke drone
    % Hindari pembagian dengan nol (ketika rho_rep = 0)
    rho_obs_unit = if_else(rho_rep > 1e-6, r_rep_vec / rho_rep, MX.zeros(size(Pu)));
    
    % Tentukan besaran gaya (magnitudo) F_rep, F_mag
    
    % Term 1: [1/rho - 1/rho_eff]
    term1 = (1/rho_rep) - (1/rho_eff);
    
    % Term 2: 1/rho^2
    term2 = 1 / (rho_rep^2);
    
    % Magnitudo Gaya Repulsif sebelum thresholding, dikalikan vektor arah
    F_rep_unclipped = k_rep * term1 * term2 * rho_obs_unit;
    
    % Aplikasikan kondisi threshold (if/else CasADi) untuk menentukan gaya F_rep akhir
    % Kondisi: rho_rep <= rho_eff
    F_rep = if_else(rho_rep <= rho_eff, F_rep_unclipped, MX.zeros(size(Pu)));
    
end