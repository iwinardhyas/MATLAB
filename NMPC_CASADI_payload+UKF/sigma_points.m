% function [X, W] = sigma_points(x, P)
%     nx = length(x);
%     alpha = 1e-3; % scaling parameter
%     kappa = 0;    % secondary scaling
%     beta = 2;     % optimal for Gaussian
% 
%     lambda = alpha^2 * (nx + kappa) - nx;
%     S = chol(P, 'lower');
% 
%     % --- Bound enforcement: pastikan massa > 0.05 dan finite ---
%     if x(end) <= 0.05 || ~isfinite(x(end))
%         fprintf('[DEBUG sigma_points] Mass invalid: %.4f — reset ke 1.0\n', x(end));
%         x(end) = 1.0;
%     end
% 
%     % Generate sigma points
% %     X = [x, x + sqrt(nx + lambda) * S, x - sqrt(nx + lambda) * S];
%     % Sigma points
%     X = zeros(nx, 2*nx+1);
%     X(:,1) = x; % titik mean
% 
%     % titik +S
%     X(:, 2:nx+1) = repmat(x,1,nx) + sqrt(nx + lambda) * S;
% 
%     % titik -S
%     X(:, nx+2:end) = repmat(x,1,nx) - sqrt(nx + lambda) * S;
% 
% 
%     % --- Bound enforcement untuk semua sigma points ---
%     for i = 1:size(X,2)
%         if X(end,i) <= 0.05 || ~isfinite(X(end,i))
%             X(end,i) = 1.0;
%         end
%     end
% 
%     % Weights
%     W = ones(1, 2 * nx + 1) * (0.5 / (nx + lambda));
%     W(1) = lambda / (nx + lambda);
% end

function [X, W] = sigma_points(x, P)
% Robust sigma points generator for UKF
% - memastikan x kolom
% - memastikan ukuran P sesuai
% - regularisasi P sebelum chol
% - fallback jika chol gagal
% - enforce lower bound pada massa (state terakhir)
%
% INPUT:
%   x : (n x 1) state
%   P : (n x n) covariance
% OUTPUT:
%   X : (n x 2n+1) sigma points
%   W : (1 x 2n+1) weights

    % --- Force column and basic checks ---
    x = x(:);
    nx = numel(x);

    if ~ismatrix(P) || any(size(P) ~= [nx nx])
        error('[sigma_points] Size mismatch: length(x)=%d, size(P)=%dx%d', nx, size(P,1), size(P,2));
    end

    % --- UT params ---
    alpha = 1e-3; kappa = 0;  % typical safe choices
    lambda = alpha^2 * (nx + kappa) - nx;

    % --- Symmetrize & regularize P ---
    P = 0.5*(P + P');
    % ensure no negative eigenvalues
    [V, D] = eig(P);
    dmin = min(diag(D));
    if dmin <= 0
        jitter = (abs(dmin) + 1e-9);
        P = P + jitter * eye(nx);
    end

    % another small jitter to help chol
    P = P + 1e-12 * eye(nx);

    % --- Cholesky with fallback ---
    try
        S = chol(P, 'lower');         % S is nx x nx
    catch ME
        warning('[sigma_points] chol failed (%s). Adding jitter and retrying.', ME.message);
        S = chol(P + 1e-6*eye(nx), 'lower');
    end

    % --- Build sigma points robustly (avoid size broadcast issues) ---
    X = repmat(x, 1, 2*nx + 1);     % nx x (2n+1)
    scaling = sqrt(nx + lambda);
    X(:, 2:nx+1)   = X(:, 2:nx+1)   + scaling * S;      % +S columns
    X(:, nx+2:end) = X(:, nx+2:end) - scaling * S;      % -S columns

    % --- Enforce physical lower bound on 'mass' state (last element) ---
    % adjust mass_min sesuai kebutuhan (kg)
    mass_min = 0.05;
    for j = 1:size(X,2)
        if ~isfinite(X(end,j)) || X(end,j) < mass_min
            X(end,j) = mass_min;
        end
    end

    % --- Weights ---
    W = ones(1, 2*nx+1) * (0.5 / (nx + lambda));
    W(1) = lambda / (nx + lambda);
end
