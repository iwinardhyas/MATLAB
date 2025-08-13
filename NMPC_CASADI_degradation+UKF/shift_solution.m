function arg_w0 = shift_solution(opt_w, nx, nu, N)
    % Shifts the optimal solution for the next iteration's warm start
    % opt_w: full optimal solution vector
    % nx: number of states
    % nu: number of inputs
    % N: prediction horizon

    % Extract the first input
    u_init = opt_w(nx + 1 : nx + nu);

    % Shift states: X_1, X_2, ..., X_N, X_N
    shifted_states = [opt_w(nx+nu+1 : end); opt_w(end-nx+1 : end)];
    
    % Shift inputs: U_1, U_2, ..., U_{N-1}, U_last_one
    shifted_inputs = [opt_w(nx+nu+1 : nx+nu+nu*(N-1)); u_init]; % Use first input for last slot

    % Reconstruct the shifted w0
    arg_w0 = [shifted_states(1:nx); u_init; shifted_states(nx+1:end)]; % This is simplified
    
    % More robust shift:
    arg_w0 = zeros(size(opt_w));
    
    % Shift X states
    for k = 0:N-1
        arg_w0(k*(nx+nu)+1 : k*(nx+nu)+nx) = opt_w((k+1)*(nx+nu)+1 : (k+1)*(nx+nu)+nx);
    end
    % Shift U inputs
    for k = 0:N-2
        arg_w0(k*(nx+nu)+nx+1 : k*(nx+nu)+nx+nu) = opt_w((k+1)*(nx+nu)+nx+1 : (k+1)*(nx+nu)+nx+nu);
    end
    % Last U is the same as the first one
    arg_w0( (N-1)*(nx+nu)+nx+1 : N*(nx+nu) ) = opt_w(nx+1 : nx+nu);
end