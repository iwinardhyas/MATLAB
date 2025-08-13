% function x_dot = quad_dynamics_for_ukf(x, u, m, g, l, Ixx, Iyy, Izz)
%     % ===================================================================
%     % Quadrotor Dynamics (12D) untuk UKF
%     % State x: [px, py, pz, phi, theta, psi, vx, vy, vz, p, q, r]
%     % Input u: [F1, F2, F3, F4] (thrust masing-masing motor)
%     % ===================================================================
% 
%     % -------------------
%     % 1. Ambil state
%     % -------------------
%     px    = x(1);  % posisi x
%     py    = x(2);  % posisi y
%     pz    = x(3);  % posisi z
%     phi   = x(4);  % roll
%     theta = x(5);  % pitch
%     psi   = x(6);  % yaw
%     vx    = x(7);  % kecepatan x
%     vy    = x(8);  % kecepatan y
%     vz    = x(9);  % kecepatan z
%     p     = x(10); % angular rate roll
%     q     = x(11); % angular rate pitch
%     r     = x(12); % angular rate yaw
% 
%     % -------------------
%     % 2. Ambil input
%     % -------------------
%     F1 = u(1);
%     F2 = u(2);
%     F3 = u(3);
%     F4 = u(4);
% 
%     % Total thrust dan momen
%     T       = F1 + F2 + F3 + F4;
%     tau_phi = l * (F2 - F4);
%     tau_theta = l * (F3 - F1);
%     tau_psi   = (F1 - F2 + F3 - F4); % bisa ditambah konstanta drag jika perlu
% 
%     % -------------------
%     % 3. Rotation Matrix (tanpa eul2rotm)
%     % -------------------
%     Rz = [cos(psi) -sin(psi) 0;
%           sin(psi)  cos(psi) 0;
%           0         0        1];
%     Ry = [cos(theta) 0 sin(theta);
%           0          1 0;
%          -sin(theta) 0 cos(theta)];
%     Rx = [1 0          0;
%           0 cos(phi) -sin(phi);
%           0 sin(phi)  cos(phi)];
% 
%     R = Rz * Ry * Rx; % ZYX convention
% 
%     % -------------------
%     % 4. Translational Dynamics
%     % -------------------
%     gravity = [0; 0; -g];
%     thrust_body = [0; 0; T/m];
%     accel = gravity + R * thrust_body;
% 
%     % -------------------
%     % 5. Rotational Dynamics
%     % -------------------
%     I = diag([Ixx, Iyy, Izz]);
%     omega = [p; q; r];
%     tau   = [tau_phi; tau_theta; tau_psi];
%     omega_dot = I \ (tau - cross(omega, I * omega));
% 
%     % -------------------
%     % 6. Kinematika Orientasi
%     % -------------------
%     % Matriks transformasi dari body rates ke euler rates
%     E = [1 sin(phi)*tan(theta)  cos(phi)*tan(theta);
%          0 cos(phi)             -sin(phi);
%          0 sin(phi)/cos(theta)   cos(phi)/cos(theta)];
% 
%     euler_dot = E * omega;
% 
%     % -------------------
%     % 7. Bentuk Persamaan State
%     % -------------------
%     x_dot = zeros(12,1);
%     x_dot(1:3)   = [vx; vy; vz];
%     x_dot(4:6)   = euler_dot;
%     x_dot(7:9)   = accel;
%     x_dot(10:12) = omega_dot;
% 
%     % -------------------
%     % 8. Validasi agar tidak NaN
%     % -------------------
%     if any(isnan(x_dot)) || any(isinf(x_dot))
%         warning('quad_dynamics_for_ukf menghasilkan NaN/Inf, mengganti dengan nol');
%         x_dot = zeros(12,1);
%     end
% end

function x_dot = quad_dynamics_for_ukf(x, u, dt)
    % Parameter drone (langsung didefinisikan di sini)
    m = 1.0; g = 9.81; l = 0.25;
    Ixx = 0.02; Iyy = 0.02; Izz = 0.04;

    % Pastikan numerik
    x = double(x); u = double(u);

    % Ambil state
    phi = x(4); theta = x(5); psi = x(6);
    vx = x(7); vy = x(8); vz = x(9);
    p = x(10); q = x(11); r = x(12);

    % Input thrust
    F1 = u(1); F2 = u(2); F3 = u(3); F4 = u(4);

    % Rotasi
    angles = [psi, theta, phi];
    if any(~isfinite(angles))  % cek NaN atau Inf
        angles = [0, 0, 0];
    end
    try
        R = eul2rotm(angles, 'ZYX');  % pastikan urutan benar
    catch
        R = eye(3);  % fallback jika error
    end


    % Gaya total
    acc = R * [0; 0; (F1+F2+F3+F4)/m] - [0; 0; g];

    % Torsi
    tau_phi = l*(F2 - F4);
    tau_theta = l*(F3 - F1);
    tau_psi = 0.01*(F1 - F2 + F3 - F4);

    % Persamaan gerak
    x_dot = zeros(12,1);
    x_dot(1:3) = [vx; vy; vz];
    x_dot(4:6) = [p; q; r];
    x_dot(7:9) = acc;
    x_dot(10) = tau_phi/Ixx;
    x_dot(11) = tau_theta/Iyy;
    x_dot(12) = tau_psi/Izz;
end

