%% 3. Definisikan Model Dinamika Quadrotor Simbolik (Fungsi State)
% Input simbolik
x = MX.sym('x', nx, 1); % State
u = MX.sym('u', nu, 1); % Input (gaya dorong motor)

% Konversi gaya dorong motor ke gaya total dan torsi
% Ini adalah bagian penting yang menghubungkan input motor (u) ke dinamika quadrotor
% Sesuaikan dengan model aktuator quadrotor Anda
F_total = sum(u);
tau_phi = l * (u(2) - u(4));  % Torsi roll
tau_theta = l * (u(3) - u(1)); % Torsi pitch
tau_psi = 0.05 * (u(1) - u(2) + u(3) - u(4)); % Torsi yaw (koefisien d_prop dari dokumentasi)

% Dinamika Quadrotor (seperti di fungsi state Anda sebelumnya)
% Posisi (x, y, z)
px = x(1); py = x(2); pz = x(3);
% Orientasi (phi, theta, psi)
phi = x(4); theta = x(5); psi = x(6);
% Kecepatan Linear (vx, vy, vz)
vx = x(7); vy = x(8); vz = x(9);
% Kecepatan Angular (p, q, r)
p = x(10); q = x(11); r = x(12);

% Rotasi dari Body Frame ke Inertial Frame
R_b_i = rotz(psi) * roty(theta) * rotx(phi);

% Percepatan linear
accel_x = R_b_i(1,3) * F_total / m;
accel_y = R_b_i(2,3) * F_total / m;
accel_z = R_b_i(3,3) * F_total / m - g; % Gravitasi ke bawah

% Percepatan angular (persamaan Euler)
p_dot = (tau_phi + (Iyy - Izz) * q * r) / Ixx;
q_dot = (tau_theta + (Izz - Ixx) * p * r) / Iyy;
r_dot = (tau_psi + (Ixx - Iyy) * p * q) / Izz;

% State derivative (dx/dt)
xdot = [vx; vy; vz; ... % Posisi
        p + q*sin(phi)*tan(theta) + r*cos(phi)*tan(theta); ... % phi_dot
        q*cos(phi) - r*sin(phi); ... % theta_dot
        q*sin(phi)/cos(theta) + r*cos(phi)/cos(theta); ... % psi_dot
        accel_x; accel_y; accel_z; ... % Kecepatan Linear
        p_dot; q_dot; r_dot]; % Kecepatan Angular

% Fungsi CasADi untuk dinamika sistem (Continuous-time)
f = Function('f', {x, u}, {xdot});

% Diskretisasi Model (Menggunakan metode Runge-Kutta ke-4)
X = MX.sym('X', nx, 1);
U = MX.sym('U', nu, 1);
XDOT = f(X, U);
k1 = XDOT;
k2 = f(X + dt/2*k1, U);
k3 = f(X + dt/2*k2, U);
k4 = f(X + dt*k3, U);
X_next = X + dt/6*(k1 + 2*k2 + 2*k3 + k4);
F_discrete = Function('F_discrete', {X, U}, {X_next});