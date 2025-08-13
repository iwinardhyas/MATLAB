function [x_est, P] = ukf_update(x_pred, P_pred, measurement, measurement_fcn, R, alpha, kappa, beta)
    % UKF Update Step
    % x_pred: Predicted state estimate
    % P_pred: Predicted covariance matrix
    % measurement: Actual sensor measurement (z)
    % measurement_fcn: Function handle for measurement model h(x)
    % R: Measurement noise covariance

    nx = length(x_pred); % Number of states

    % Generate Sigma Points for predicted state
    lambda = alpha^2 * (nx + kappa) - nx;
    chol_P_pred = chol((nx + lambda) * P_pred, 'lower');
    
    sigma_points_pred = zeros(nx, 2*nx + 1);
    sigma_points_pred(:, 1) = x_pred;
    for i = 1:nx
        sigma_points_pred(:, i+1)   = x_pred + chol_P_pred(:, i);
        sigma_points_pred(:, i+1+nx) = x_pred - chol_P_pred(:, i);
    end

    % Weights for mean and covariance (same as predict step)
    Wm = [lambda / (nx + lambda); 0.5 / (nx + lambda) * ones(2*nx, 1)];
    Wc = Wm;
    Wc(1) = Wm(1) + (1 - alpha^2 + beta);

    % Propagate sigma points through measurement function
    sigma_measurements = zeros(length(measurement), 2*nx + 1);
    for i = 1:(2*nx + 1)
        sigma_measurements(:, i) = measurement_fcn(sigma_points_pred(:, i));
    end

    % Calculate predicted measurement mean (y_hat)
    y_hat = sigma_measurements * Wm;

    % Calculate measurement covariance (P_yy)
    P_yy = zeros(length(measurement), length(measurement));
    for i = 1:(2*nx + 1)
        error = sigma_measurements(:, i) - y_hat;
        P_yy = P_yy + Wc(i) * (error * error');
    end
    P_yy = P_yy + R; % Add measurement noise covariance

    % Calculate cross-covariance (P_xy)
    P_xy = zeros(nx, length(measurement));
    for i = 1:(2*nx + 1)
        x_error = sigma_points_pred(:, i) - x_pred;
        y_error = sigma_measurements(:, i) - y_hat;
        P_xy = P_xy + Wc(i) * (x_error * y_error');
    end

    % Kalman Gain
    K = P_xy / P_yy;

    % Update state and covariance
    x_est = x_pred + K * (measurement - y_hat);
    P = P_pred - K * P_yy * K';
end