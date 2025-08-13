function S = cholupdate_qr(S_prev, X_diff, Wc, Q)
    % Bobot sigma point
    X_weighted = X_diff .* sqrt(Wc(:)');
    
    % Pastikan Q positif-definit
    Q = (Q + Q')/2;
    [V, D] = eig(Q);
    D(D < 1e-8) = 1e-8;
    Q = V * D * V';
    
    % Cholesky aman
    try
        chol_Q = chol(Q, 'lower');
    catch
        chol_Q = chol(Q + 1e-6*eye(size(Q)), 'lower');
    end
    
    % Bentuk matriks gabungan
    A = [X_weighted, chol_Q];
    
    % QR decomposition
    [~, R] = qr(A', 0);
    S = R';
    
    % Regularisasi tambahan
    S = (S + S')/2;
    S = S + 1e-6 * eye(size(S));
end
