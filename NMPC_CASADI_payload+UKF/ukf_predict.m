function [x_pred, P_pred] = ukf_predict(x_est, P, u_applied, dynamics_fcn, Q, dt, g_val, alpha, kappa, beta)
    % UKF Prediction Step
    % x_est: Current state estimate [z; vz; m]
    % P: Current covariance matrix
    % u_applied: Control input (TotalThrust_z)
    % dynamics_fcn: Function handle for continuous-time dynamics f(x, u, g)
    % Q: Process noise covariance
    % dt: Time step
    % g_val: Gravity
    % alpha, kappa, beta: UKF parameters

    nx = length(x_est); % Number of states

    % Generate Sigma Points
    lambda = alpha^2 * (nx + kappa) - nx;
    chol_P = chol((nx + lambda) * P, 'lower'); % Cholesky decomposition of scaled covariance
    
    sigma_points = zeros(nx, 2*nx + 1);
    sigma_points(:, 1) = x_est; % Mean point
    for i = 1:nx
        sigma_points(:, i+1)   = x_est + chol_P(:, i);
        sigma_points(:, i+1+nx) = x_est - chol_P(:, i);
    end

    % Weights for mean and covariance
    Wm = [lambda / (nx + lambda); 0.5 / (nx + lambda) * ones(2*nx, 1)];
    Wc = Wm;
    Wc(1) = Wm(1) + (1 - alpha^2 + beta); % Adjust weight for covariance of mean point

    % Propagate Sigma Points through non-linear dynamics
    sigma_points_propagated = zeros(nx, 2*nx + 1);
    for i = 1:(2*nx + 1)
        % Integrate each sigma point
        current_sigma_x = sigma_points(:, i);
        
        % Use RK4 for integration (simplified here, you'd use your actual drone model)
        k1 = dynamics_fcn(current_sigma_x, u_applied, g_val);
        k2 = dynamics_fcn(current_sigma_x + dt/2 * k1, u_applied, g_val);
        k3 = dynamics_fcn(current_sigma_x + dt/2 * k2, u_applied, g_val);
        k4 = dynamics_fcn(current_sigma_x + dt * k3, u_applied, g_val);
        
        sigma_points_propagated(:, i) = current_sigma_x + dt/6 * (k1 + 2*k2 + 2*k3 + k4);
    end

    % Calculate predicted mean and covariance
    x_pred = sigma_points_propagated * Wm; % Weighted sum of propagated sigma points

    P_pred = zeros(nx, nx);
    for i = 1:(2*nx + 1)
        error = sigma_points_propagated(:, i) - x_pred;
        P_pred = P_pred + Wc(i) * (error * error');
    end
    P_pred = P_pred + Q; % Add process noise covariance

end