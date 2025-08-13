function [X, W] = sigma_points(x, P)
    nx = length(x);
    lambda = 1; 
    S = chol(P,'lower');
    X = [x, x + sqrt(nx+lambda)*S, x - sqrt(nx+lambda)*S];
    W0 = lambda/(nx+lambda);
    Wi = 1/(2*(nx+lambda));
    W = [W0, Wi*ones(1,2*nx)];  % 1x(2*nx+1)
end