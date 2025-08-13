Q_mass_values = logspace(-6, -2, 5);   % 1e-6 hingga 1e-2
Q_vz_values = logspace(-6, -2, 5);
Q_z_values = logspace(-6, -2, 5);
R_z_values = logspace(-3, -1, 5);      % 1e-3 hingga 1e-1
RMSE_matrix = zeros(length(Q_mass_values), length(R_z_values));
% --- Inisialisasi Penghitung Progres dan Timer ---
total_iterations = length(Q_z_values) * length(Q_vz_values) * ...
                   length(Q_mass_values) * length(R_z_values);
current_iteration = 0;

fprintf('Memulai proses tuning UKF...\n');
fprintf('Total simulasi yang akan dijalankan: %d\n', total_iterations);

tic; % Mulai timer untuk seluruh proses tuning

for i = 1:length(Q_z_values)
    for j = 1:length(Q_vz_values)
        for k = 1:length(Q_mass_values)
            for l = 1:length(R_z_values)
                 current_iteration = current_iteration + 1; % Tingkatkan penghitung iterasi
                % --- Konfigurasi Parameter ---
                Q_mass = Q_mass_values(k);
                Q_z = Q_z_values(i);
                Q_vz = Q_vz_values(j);
                R_z = R_z_values(l);

                % Set Q dan R UKF
                ukf_Q = diag([Q_z, Q_vz, Q_mass]);   % Tetap kecil untuk z dan vz
                ukf_R = diag([R_z]);

                % --- Jalankan Simulasi (fungsi terpisah) ---
                % Harus return: estimasi massa & massa aktual
                [estimated_mass, actual_mass] = run_UKF_simulation(ukf_Q, ukf_R);

                % --- Hitung Error Estimasi ---
                RMSE = sqrt(mean((estimated_mass - actual_mass).^2));
                RMSE_matrix(i, j, k, l) = RMSE;
                
                % --- Tampilkan Progres ke Konsol ---
                elapsed_time = toc; % Waktu yang telah berlalu sejak awal proses tuning
                avg_time_per_iteration = elapsed_time / current_iteration;
                remaining_iterations = total_iterations - current_iteration;
                estimated_time_remaining = avg_time_per_iteration * remaining_iterations;

                % Konversi waktu ke format HH:MM:SS
                [eh, em, es] = sec2hms(elapsed_time);
                [rh, rm, rs] = sec2hms(estimated_time_remaining);

                progress_percent = (current_iteration / total_iterations) * 100;

                fprintf('Progres: %d/%d (%.2f%%) | Berjalan: %02d:%02d:%02d | Sisa: %02d:%02d:%02d | RMSE: %.4f (Q_m %.2e, Q_v %.2e, Q_z %.2e, R_z %.2e)\n', ...
                        current_iteration, total_iterations, progress_percent, ...
                        eh, em, es, rh, rm, rs, RMSE, Q_mass, Q_vz, Q_z, R_z);
            end
        end
    end
end

% --- Buat Heatmap ---
% figure;
% imagesc(log10(R_z_values), log10(Q_mass_values), RMSE_matrix);
% xlabel('log_{10}(R_z)');
% ylabel('log_{10}(Q_{mass})');
% title('Heatmap RMSE Estimasi Massa UKF');
% colorbar;
% set(gca, 'YDir', 'normal'); % agar Y dari bawah ke atas

fprintf('Proses tuning selesai!\n');
total_tuning_time = toc; % Waktu total untuk seluruh proses tuning
fprintf('Total waktu tuning: %02d:%02d:%02d\n', sec2hms(total_tuning_time));

[min_val, min_idx] = min(RMSE_matrix(:));
[i_opt, j_opt, k_opt, l_opt] = ind2sub(size(RMSE_matrix), min_idx);

optimal_Qmass = Q_mass_values(k_opt);
optimal_Qz = Q_z_values(i_opt);
optimal_Qvz = Q_vz_values(j_opt);
optimal_Rz = R_z_values(l_opt);

fprintf('Nilai optimal ditemukan:\n');
fprintf('Q_mass = %.2e (log10=%.2f)\n', optimal_Qmass, log10(optimal_Qmass));
fprintf('Q_vz = %.2e (log10=%.2f)\n', optimal_Qvz, log10(optimal_Qvz));
fprintf('Q_z = %.2e (log10=%.2f)\n', optimal_Qz, log10(optimal_Qz));
fprintf('R_z    = %.2e (log10=%.2f)\n', optimal_Rz, log10(optimal_Rz));
fprintf('RMSE terkecil = %.4f\n', min_val);

% --- Fungsi Pembantu untuk Konversi Detik ke HH:MM:SS ---
function [h, m, s] = sec2hms(total_seconds)
    h = floor(total_seconds / 3600);
    total_seconds = total_seconds - h * 3600;
    m = floor(total_seconds / 60);
    s = total_seconds - m * 60;
    s = round(s); % Bulatkan detik
end
