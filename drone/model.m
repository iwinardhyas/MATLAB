function [sys, Ts] = model()
    % MODEL.M - Membuat model state-space quadcopter
    
    % Matriks State-Space
    A = [0 1 0 0; 
         0 0 -1 0; 
         0 0 0 1; 
         0 0 0 0];  % Matriks A
    B = [0; 0; 0; 1];                % Matriks B (input)
    C = eye(4);                      % Matriks C (output langsung adalah state)
    D = zeros(4, 1);                 % Matriks D
    
    Ts = 0.01;                       % Waktu sampling
    sys = ss(A, B, C, D);            % Sistem state-space kontinu
    
    % Diskretkan sistem jika diperlukan
    sys = c2d(sys, Ts);
end
