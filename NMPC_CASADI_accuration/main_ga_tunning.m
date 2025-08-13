clear; clc;

% Ambil nilai Q dan R terbaik dari hasil sebelumnya
Q_prev_best = [5.8428 1.0835 6.4114 2.3008 5.0594 4.5821 8.6859 0.0485 4.7640 7.7026 6.6228 2.7771];
R_prev_best = [0.5478 6.7010 0.0798 8.6135];

% Misal lebar range 40% dari nilai (±20%)
Q_lower = max(Q_prev_best - 0.2*Q_prev_best, 0.001);  % tetap jaga batas bawah > 0
Q_upper = Q_prev_best + 0.2*Q_prev_best;

R_lower = max(R_prev_best - 0.2*R_prev_best, 0.001);
R_upper = R_prev_best + 0.2*R_prev_best;

% Set range baru
search_range.Q = [Q_lower; Q_upper];
search_range.R = [R_lower; R_upper];


% nQ = 12;  % state dimension
% nR = 4;   % control dimension
% 
% search_range.Q = [1e-3*ones(1,nQ); 10*ones(1,nQ)];
% search_range.R = [1e-3*ones(1,nR); 10*ones(1,nR)];

pop_size = 100;
max_gen = 100;
crossover_rate = 0.8;
mutation_rate = 0.05;

[Q_tuned, R_tuned, ga_history] = ga_tune_QR(@nmpc_casadi_ga, search_range, pop_size, max_gen, crossover_rate, mutation_rate);

disp('Q terbaik yang ditemukan:');
disp(Q_tuned);
disp('R terbaik yang ditemukan:');
disp(R_tuned);
