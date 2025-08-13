function cost = fitness_QR(Q_vec, R_vec)
    try
        % Panggil NMPC dengan Q dan R (dalam bentuk diagonal)
        Q = diag(Q_vec);
        R = diag(R_vec);
        tracking_error = nmpc_casadi_ga(Q, R);  % harus return total error atau RMSE
        cost = tracking_error;  % Semakin kecil semakin baik
    catch
        cost = 1e6;  % penalti jika gagal
    end
end
