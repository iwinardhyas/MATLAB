% Parameter
k_rep = 15.0;      % gain repulsif
d0    = 8.0;      % radius pengaruh obstacle
d     = linspace(0.1, 10, 500);   % jarak ke obstacle

% --- APF klasik ---
U_classic = 0.5 * k_rep * (1./d - 1/d0).^2;
U_classic(d > d0) = 0;  % diluar radius nol
F_classic = k_rep * (1./d - 1/d0) .* (1./d.^2);
F_classic(d > d0) = 0;

% --- APF usulan (squared-inverse) ---
U_mod = 0.5 * k_rep * (1./d - 1/d0).^2 .* (d0./d);
U_mod(d > d0) = 0;
% gradien numerik pakai diff
F_mod = -gradient(U_mod, d);

% Plot gaya repulsif
figure;
plot(d, F_classic, 'b-', 'LineWidth', 2); hold on;
plot(d, F_mod, 'r--', 'LineWidth', 2);
xlabel('Jarak ke obstacle d');
ylabel('||F(d)||');
legend('APF Klasik','APF Usulan (Squared-Inverse)');
title('Perbandingan gaya repulsif APF');
grid on;

% Plot fungsi potensial
figure;
plot(d, U_classic, 'b-', 'LineWidth', 2); hold on;
plot(d, U_mod, 'r--', 'LineWidth', 2);
xlabel('Jarak ke obstacle d');
ylabel('U(d)');
legend('Potensial Klasik','Potensial Usulan');
title('Perbandingan fungsi potensial APF');
grid on;
