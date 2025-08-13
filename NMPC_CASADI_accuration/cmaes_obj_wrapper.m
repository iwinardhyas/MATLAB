% cmaes_obj_wrapper.m
function f = cmaes_obj_wrapper(x)
    % Wrapper fungsi untuk CMA-ES
    nQ = 12;
    nR = 4;
    persistent eval_count
    if isempty(eval_count), eval_count = 0; end
    eval_count = eval_count + 1;

    fprintf('Evaluasi ke-%d, Q=%.3f, R=%.3f\n', eval_count, x(1), x(end));
    f = nmpc_cma_wrapper(x, nQ, nR);
end
