function S_new = cholupdate_multi(S, U, sign_flag)
    % Rank-one Cholesky update/downdate
    % S: lower Cholesky factor
    % U: update vector(s)
    % sign_flag: +1 for update, -1 for downdate
    
    if size(U,2) > 1
        for k = 1:size(U,2)
            S = cholupdate_multi(S, U(:,k), sign_flag);
        end
        S_new = S;
        return;
    end
    
    x = U;
    for k = 1:size(S,1)
        r = sqrt(S(k,k)^2 + sign_flag*x(k)^2);
        c = r / S(k,k);
        s = x(k) / S(k,k);
        S(k,k) = r;
        if k < size(S,1)
            S(k+1:end,k) = (S(k+1:end,k) + sign_flag*s*x(k+1:end)) / c;
            x(k+1:end)   = c*x(k+1:end) - s*S(k+1:end,k);
        end
    end
    S_new = S;
end
