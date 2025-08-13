% Parameter Quadcopter
m = 1.26;          % massa (kg)
g = 9.8;           % percepatan gravitasi (m/s^2)
Ix = 0.00168;      % momen inersia sumbu x (kg.m^2)
Iy = 0.00168;      % momen inersia sumbu y (kg.m^2)
Iz = 0.00125;      % momen inersia sumbu z (kg.m^2)
kT = 0.00000168918; % konstanta thrust (N.s^2)
kD = 0.0000419;    % konstanta drag (Nm.s^2)

% Titik Kerja
F_T_eq = m * g; % Gaya thrust pada kondisi setimbang

% Matriks Rotasi (untuk titik kerja)
phi = 0; theta = 0; psi = 0; % Euler angles (titik kerja)
R = [cos(theta)*cos(psi),                               cos(theta)*sin(psi),                                -sin(theta); 
     sin(phi)*sin(theta)*cos(psi)-cos(phi)*sin(psi),    sin(phi)*sin(theta)*sin(psi)+cos(phi)*cos(psi),     sin(phi)*cos(theta);
     cos(phi)*sin(theta)*cos(psi)+sin(phi)*sin(psi),     cos(phi)*sin(theta)*sin(psi)-sin(phi)*cos(psi),    cos(phi)*cos(theta)];

% Sistem Translasi
A_trans = zeros(6, 6);
A_trans(1:3, 4:6) = eye(3);
B_trans = zeros(6, 4);
B_trans(4:6, 1) = [0; 0; 1/m];

% Sistem Rotasi
A_rot = zeros(6, 6);
A_rot(1:3, 4:6) = eye(3);
B_rot = zeros(6, 4);

% Matriks Momen Inersia
I = diag([Ix, Iy, Iz]);

% Matriks Input Rotasi
B_rot(4:6, 2:4) = inv(I);

% Matriks Linier Keseluruhan
A = blkdiag(A_trans, A_rot); % Gabungan translasi dan rotasi
B = [B_trans; B_rot];        % Gabungan input translasi dan rotasi

% Matriks C dan D (Output)
C = eye(size(A)); % Semua state diamati
D = zeros(size(B)); % Tidak ada hubungan langsung input-output

% Menampilkan Hasil
disp('Matriks A:');
disp(A);

disp('Matriks B:');
disp(B);

disp('Matriks C:');
disp(C);

disp('Matriks D:');
disp(D);
