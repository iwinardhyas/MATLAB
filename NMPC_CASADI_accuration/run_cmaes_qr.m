% run_cmaes_qr.m
clc; clear;

num_q = 12;
num_r = 4;
num_params = num_q + num_r;

% Fungsi objektif dibungkus
fitfun = 'cmaes_obj_wrapper';

q_min = 0.01; q_max = 100;
r_min = 0.0001; r_max = 10;

lb = [q_min * ones(num_q,1); r_min * ones(num_r,1)];
ub = [q_max * ones(num_q,1); r_max * ones(num_r,1)];

% Titik awal dan deviasi awal
x0 = (lb + ub) / 2;
sigma = 0.3 * (ub - lb);  % deviasi awal: 30% dari rentang

% Opsi CMA-ES
opts = cmaes('defaults');
opts.LBounds = lb;
opts.UBounds = ub;
opts.PopSize = 16;
opts.MaxIter = 50;
opts.StopFitness = 1e-6;    % stop jika fitness sudah cukup bagus
opts.DispFinal = 'on';
opts.SaveVariables = 'off';
opts.LogModulo = 1;
opts.Seed = 123;
opts.DispModulo = 10;       % tampilkan setiap 10 iterasi

% Jalankan CMA-ES
tStart = tic;

[best_params, best_fval, counteval, stopflag, out, bestever] = ...
    cmaes(fitfun, x0, sigma, opts);

tElapsed = toc(tStart);



% Ambil Q dan R terbaik
Q_best = best_params(1:nQ);
R_best = best_params(nQ+1:end);

fprintf('Q terbaik yang ditemukan:\n');
disp(Q_best);
fprintf('R terbaik yang ditemukan:\n');
disp(R_best);

disp('--- Hasil Optimasi ---');
fprintf('Best cost: %.6f\n', best_fval);
disp('Best Q values:');
disp(best_params(1:num_q)');
disp('Best R values:');
disp(best_params(num_q+1:end)');
fprintf('Waktu total optimasi: %.2f detik\n', tElapsed);
fprintf('Evaluasi fungsi: %d\n', counteval);

Q_opt = diag(Q_best);
R_opt = diag(R_best);

disp('Q optimal:'); disp(Q_opt);
disp('R optimal:'); disp(R_opt);