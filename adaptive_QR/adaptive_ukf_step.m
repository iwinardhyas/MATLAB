function [x_est, P_est, Q_est, R_est] = adaptive_ukf_step(x_prev, P_prev, u_input, y_meas, ...
    f_dyn, h_meas, Q_prev, R_prev, dt, alpha, kappa, beta, lambda_Q, lambda_R)

    % ---------------------------
    % 1. UKF Prediction Step
    % ---------------------------
    [x_pred, P_pred, X_sigma_pred] = ukf_predict(x_prev, P_prev, u_input, f_dyn, Q_prev, dt, alpha, kappa, beta);

    % ---------------------------
    % 2. UKF Measurement Update
    % ---------------------------
    [x_upd, P_upd, y_pred, Pyy, Pxy] = ukf_update(x_pred, P_pred, y_meas, h_meas, R_prev, alpha, kappa, beta);

    % ---------------------------
    % 3. Adaptive Q & R Update
    % ---------------------------
    innovation = y_meas - y_pred;
    prediction_error = x_upd - x_pred;

    % Adaptive Measurement Noise (R)
    R_est = (1 - lambda_R) * R_prev + lambda_R * (innovation * innovation' + Pyy);

    % Adaptive Process Noise (Q)
    Q_est = (1 - lambda_Q) * Q_prev + lambda_Q * (prediction_error * prediction_error');

    % Output
    x_est = x_upd;
    P_est = P_upd;
end
