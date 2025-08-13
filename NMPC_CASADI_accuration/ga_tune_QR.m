%     function [Q_best, R_best, history] = ga_tune_QR(fitness_func, search_range, pop_size, max_gen)
% 
%         nQ = size(search_range.Q, 2);
%         nR = size(search_range.R, 2);
%         dim = nQ + nR;
% 
%         % Inisialisasi populasi awal (acak dalam rentang pencarian)
%         pop = rand(pop_size, dim);
%         for i = 1:nQ
%             pop(:, i) = search_range.Q(1, i) + pop(:, i)*(search_range.Q(2, i) - search_range.Q(1, i));
%         end
%         for i = 1:nR
%             idx = nQ + i;
%             pop(:, idx) = search_range.R(1, i) + pop(:, idx)*(search_range.R(2, i) - search_range.R(1, i));
%         end
% 
%         % Evaluasi fitness awal
%         fitness = zeros(pop_size, 1);
%         for i = 1:pop_size
%             Q_vec = pop(i, 1:nQ);
%             R_vec = pop(i, nQ+1:end);
%             fitness(i) = fitness_func(Q_vec, R_vec);  % INI YANG BENAR
%         end
% 
%         % Simpan riwayat terbaik
%         [~, best_idx] = min(fitness);
%         Q_best = pop(best_idx, 1:nQ);
%         R_best = pop(best_idx, nQ+1:end);
%         history = zeros(max_gen, 1);
%         history(1) = fitness(best_idx);
% 
%         % Iterasi generasi GA
%         for gen = 2:max_gen
%             % Seleksi (tournament)
%             parents_idx = randi(pop_size, pop_size, 2);
%             new_pop = zeros(size(pop));
%             for i = 1:pop_size
%                 if fitness(parents_idx(i,1)) < fitness(parents_idx(i,2))
%                     winner = parents_idx(i,1);
%                 else
%                     winner = parents_idx(i,2);
%                 end
%                 new_pop(i,:) = pop(winner,:);
%             end
% 
%             % Crossover
%             for i = 1:2:pop_size-1
%                 alpha = rand;
%                 new_pop([i i+1],:) = alpha*new_pop([i i+1],:) + (1-alpha)*flipud(new_pop([i i+1],:));
%             end
% 
%             % Mutasi
%             mutation_rate = 0.1;
%             for i = 1:pop_size
%                 if rand < mutation_rate
%                     mut_idx = randi(dim);
%                     if mut_idx <= nQ
%                         new_pop(i, mut_idx) = search_range.Q(1, mut_idx) + rand*(search_range.Q(2, mut_idx) - search_range.Q(1, mut_idx));
%                     else
%                         r_idx = mut_idx - nQ;
%                         new_pop(i, mut_idx) = search_range.R(1, r_idx) + rand*(search_range.R(2, r_idx) - search_range.R(1, r_idx));
%                     end
%                 end
%             end
% 
%             % Evaluasi populasi baru
%             pop = new_pop;
%             for i = 1:pop_size
%                 Q_vec = pop(i, 1:nQ);
%                 R_vec = pop(i, nQ+1:end);
%                 fitness(i) = fitness_func(Q_vec, R_vec);
%             end
% 
%             % Simpan solusi terbaik
%             [~, best_idx] = min(fitness);
%             Q_best = pop(best_idx, 1:nQ);
%             R_best = pop(best_idx, nQ+1:end);
%             history(gen) = fitness(best_idx);
%             save('checkpoint_best_QR.mat', 'Q_best', 'R_best', 'history');
%         end
%     end

function [Q_best, R_best, history] = ga_tune_QR(fitness_func, search_range, pop_size, max_gen, crossover_rate, mutation_rate)

    % Input:
    %   fitness_func    : Fungsi handle yang menerima (Q_vec, R_vec) dan mengembalikan nilai fitness (semakin kecil semakin baik)
    %   search_range    : Struct dengan field Q dan R. Contoh:
    %                     search_range.Q = [Q_min_values; Q_max_values]; % 2x12 matrix
    %                     search_range.R = [R_min_values; R_max_values]; % 2x4 matrix
    %   pop_size        : Ukuran populasi
    %   max_gen         : Jumlah generasi maksimum
    %   crossover_rate  : Probabilitas crossover (e.g., 0.8)
    %   mutation_rate   : Probabilitas mutasi (e.g., 0.05)

    nQ = size(search_range.Q, 2); % Jumlah elemen di vektor Q (misal 12)
    nR = size(search_range.R, 2); % Jumlah elemen di vektor R (misal 4)
    dim = nQ + nR; % Total dimensi individu (16)

    % Inisialisasi populasi awal (acak dalam rentang pencarian)
    fprintf('Inisialisasi populasi awal...\n');
    pop = rand(pop_size, dim);
    for i = 1:nQ
        pop(:, i) = search_range.Q(1, i) + pop(:, i) .* (search_range.Q(2, i) - search_range.Q(1, i));
    end
    for i = 1:nR
        idx = nQ + i;
        pop(:, idx) = search_range.R(1, i) + pop(:, idx) .* (search_range.R(2, i) - search_range.R(1, i));
    end

    % Evaluasi fitness awal
    fitness = zeros(pop_size, 1);
    fprintf('Mengevaluasi fitness populasi awal...\n');
    parfor i = 1:pop_size % Gunakan parfor jika Anda memiliki Parallel Computing Toolbox
        Q_vec = pop(i, 1:nQ);
        R_vec = pop(i, nQ+1:end);
        fitness(i) = fitness_func(Q_vec, R_vec);
        fprintf('  Evaluasi individu %d/%d: Fitness = %.4f\n', i, pop_size, fitness(i));
    end

    % Simpan riwayat fitness
    history.best_fitness = zeros(max_gen, 1);
    history.avg_fitness = zeros(max_gen, 1);
    
    [best_fitness_gen, best_idx] = min(fitness);
    Q_best = pop(best_idx, 1:nQ);
    R_best = pop(best_idx, nQ+1:end);
    history.best_fitness(1) = best_fitness_gen;
    history.avg_fitness(1) = mean(fitness);

    fprintf('Generasi 1: Fitness Terbaik = %.4f, Rata-rata = %.4f\n', ...
            history.best_fitness(1), history.avg_fitness(1));

    % Iterasi generasi GA
    total_start_time = tic; % Mulai hitung total waktu
    for gen = 2:max_gen
        gen_start_time = tic; % Mulai hitung waktu per generasi

        % Simpan individu terbaik dari generasi saat ini (untuk Elitisme)
        [current_best_fitness, current_best_idx] = min(fitness);
        elite_individual = pop(current_best_idx, :);

        % Seleksi (Tournament Selection)
        new_pop = zeros(size(pop));
        for i = 1:pop_size
            idx1 = randi(pop_size);
            idx2 = randi(pop_size);
            % Pastikan individu yang dipilih berbeda
            while idx1 == idx2
                idx2 = randi(pop_size);
            end
            if fitness(idx1) < fitness(idx2)
                winner = idx1;
            else
                winner = idx2;
            end
            new_pop(i,:) = pop(winner,:);
        end

        % Crossover (Arithmetic Crossover)
        for i = 1:2:pop_size-1
            if rand < crossover_rate
                alpha = rand; % Nilai alpha acak untuk setiap pasangan
                
                % Dapatkan parent (dari new_pop hasil seleksi)
                parent1 = new_pop(i,:);
                parent2 = new_pop(i+1,:);

                % Lakukan crossover
                offspring1 = alpha * parent1 + (1 - alpha) * parent2;
                offspring2 = alpha * parent2 + (1 - alpha) * parent1;

                new_pop(i,:) = offspring1;
                new_pop(i+1,:) = offspring2;
            end
        end
        
        % Penanganan Batas setelah Crossover
        for k_idx = 1:nQ
            new_pop(:, k_idx) = max(search_range.Q(1, k_idx), min(search_range.Q(2, k_idx), new_pop(:, k_idx)));
        end
        for k_idx = 1:nR
            col_idx = nQ + k_idx;
            new_pop(:, col_idx) = max(search_range.R(1, k_idx), min(search_range.R(2, k_idx), new_pop(:, col_idx)));
        end

        % Mutasi
        for i = 1:pop_size
            for j = 1:dim % Mutasi setiap gen dengan probabilitas mutation_rate
                if rand < mutation_rate
                    if j <= nQ % Mutasi gen Q
                        new_pop(i, j) = search_range.Q(1, j) + rand * (search_range.Q(2, j) - search_range.Q(1, j));
                    else % Mutasi gen R
                        r_idx = j - nQ;
                        new_pop(i, j) = search_range.R(1, r_idx) + rand * (search_range.R(2, r_idx) - search_range.R(1, r_idx));
                    end
                end
            end
        end

        % Elitisme: Ganti individu terburuk di populasi baru dengan individu terbaik dari generasi sebelumnya
        [~, worst_idx_new_pop] = max(fitness); % Temukan indeks individu terburuk
        new_pop(worst_idx_new_pop, :) = elite_individual; % Ganti dengan individu elit

        % Evaluasi populasi baru
        pop = new_pop;
        fprintf('Mengevaluasi fitness generasi %d...\n', gen);
        parfor i = 1:pop_size % Gunakan parfor
            Q_vec = pop(i, 1:nQ);
            R_vec = pop(i, nQ+1:end);
            fitness(i) = fitness_func(Q_vec, R_vec);
            % fprintf('  Evaluasi individu %d/%d: Fitness = %.4f\n', i, pop_size, fitness(i)); % Terlalu verbose untuk loop ini
        end

        % Simpan solusi terbaik dan statistik
        [best_fitness_gen, best_idx] = min(fitness);
        Q_best = pop(best_idx, 1:nQ);
        R_best = pop(best_idx, nQ+1:end);
        history.best_fitness(gen) = best_fitness_gen;
        history.avg_fitness(gen) = mean(fitness);
        
        gen_elapsed_time = toc(gen_start_time); % Waktu yang berlalu untuk generasi ini
        fprintf('Generasi %d: Fitness Terbaik = %.4f, Rata-rata = %.4f, Waktu = %.2f detik\n', ...
                gen, history.best_fitness(gen), history.avg_fitness(gen), gen_elapsed_time);
        
        % Simpan checkpoint setiap generasi
        save('checkpoint_best_QR.mat', 'Q_best', 'R_best', 'history');
    end

    total_elapsed_time = toc(total_start_time); % Total waktu yang berlalu
    fprintf('\nOptimasi GA selesai. Total waktu: %.2f detik\n', total_elapsed_time);

    %% Visualisasi Grafik Konvergensi
    figure;
    plot(1:max_gen, history.best_fitness, 'b-', 'LineWidth', 2, 'DisplayName', 'Fitness Terbaik');
    hold on;
    plot(1:max_gen, history.avg_fitness, 'r--', 'LineWidth', 1.5, 'DisplayName', 'Fitness Rata-rata');
    xlabel('Generasi');
    ylabel('Nilai Fitness');
    title('Konvergensi Algoritma Genetika');
    legend('Location', 'best');
    grid on;
    hold off;

end