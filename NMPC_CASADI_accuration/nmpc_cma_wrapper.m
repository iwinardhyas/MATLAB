function total_error = nmpc_cma_wrapper(param_vec, nQ, nR)
    % Split parameter vector menjadi Q dan R
    Q_vec = param_vec(1:nQ);
    R_vec = param_vec(nQ+1:end);
    
    % Panggil fungsi evaluasi NMPC
    total_error = nmpc_casadi_ga(Q_vec, R_vec);
end
