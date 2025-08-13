function [X_sigma, Wm, Wc] = sigma_points(x, P, lambda, n)
    U = chol((n+lambda)*P, 'lower');
    X_sigma = [x, x + U, x - U];
    Wm = [lambda/(n+lambda), 0.5/(n+lambda)*ones(1,2*n)];
    Wc = Wm;
    Wc(1) = Wc(1) + (1 - alpha^2 + beta);
end
