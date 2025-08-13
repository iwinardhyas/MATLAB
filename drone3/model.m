function [sys, Ts] = model()
    % MODEL.M - Membuat model state-space quadcopter
    
    % Matriks State-Space
    A = [0 0 0 1 0 0; 
         0 0 0 0 1 0; 
         0 0 0 0 0 1; 
         0 0 0 -0.1 0 0;
         0 0 0 0 -0.1 0;
         0 0 0 0 0 -0.1];  % Matriks A
    B = [0 0 0;
        0 0 0;
        0 0 0;
        1 0 0;
        0 1 0;
        0 0 1];                % Matriks B (input)
%     C = [0 0 0 0 0 0; 
%          0 0 0 0 0 0;
%          0 0 0 0 0 0;
%          1 0 0 0 0 0;
%          0 1 0 0 0 0;
%          0 0 1 0 0 0];                      % Matriks C (output langsung adalah state)
    C = eye(6); % Output all states
    D = 0;                 % Matriks D
    
    Ts = 0.01;                       % Waktu sampling
    sys = ss(A, B, C, D);            % Sistem state-space kontinu
    
    % Diskretkan sistem jika diperlukan
    sys = c2d(sys, Ts);
end
