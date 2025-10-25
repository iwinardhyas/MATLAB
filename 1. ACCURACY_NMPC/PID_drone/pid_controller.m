function u_pid = pid_controller(x_current, x_target, params, Kp, Ki, Kd, PID_I_state)
% PID Controller untuk Quadrotor - Cascade Control
% Fixed version dengan formula yang benar

%% Parameter Fisik
massa = params.massa;
gravitasi = params.gravitasi;
l = params.l;
hover_thrust = massa * gravitasi; 
Mixer_Inv = params.Mixer_Inv;

%% ===== OUTER LOOP: Position Control =====
% Error Posisi dan Kecepatan
error_pos = x_target(1:3) - x_current(1:3);      % [ex, ey, ez]
error_vel = x_target(7:9) - x_current(7:9);      % [evx, evy, evz]

% --- 1. Altitude Control (Z-axis) ---
% PID untuk ketinggian
u_z = Kp(3) * error_pos(3) + Ki(3) * PID_I_state(3) + Kd(3) * error_vel(3);

% Total thrust yang dibutuhkan (dalam Newton)
F_z_target = massa * (gravitasi + u_z);

% Batasi thrust total agar realistis
F_z_min = massa * gravitasi * 0.5;  % Minimum 50% hover (bisa turun)
F_z_max = massa * gravitasi * 2.0;  % Maximum 200% hover (bisa naik cepat)
F_z_target = max(F_z_min, min(F_z_max, F_z_target));

% --- 2. Horizontal Control (X, Y) ---
% PID untuk posisi horizontal → akselerasi target
ax_des = Kp(1) * error_pos(1) + Ki(1) * PID_I_state(1) + Kd(1) * error_vel(1);
ay_des = Kp(2) * error_pos(2) + Ki(2) * PID_I_state(2) + Kd(2) * error_vel(2);

% Batasi akselerasi horizontal (mencegah manuver terlalu agresif)
a_max = 3.0; % m/s² (sekitar 0.3g)
ax_des = max(-a_max, min(a_max, ax_des));
ay_des = max(-a_max, min(a_max, ay_des));

% --- Konversi Akselerasi ke Target Roll/Pitch ---
% PERBAIKAN: Formula yang benar tanpa mengalikan massa
% Untuk quadrotor: ax = g * tan(theta), ay = -g * tan(phi)
% Inverse: theta = atan(ax/g), phi = -atan(ay/g)

% Small angle approximation (lebih stabil untuk kontrol):
phi_target = -ay_des / gravitasi;     % Roll target (rad)
theta_target = ax_des / gravitasi;    % Pitch target (rad)

% Atau gunakan formula lengkap untuk akurasi lebih baik:
% g_eff = sqrt(gravitasi^2 + ax_des^2 + ay_des^2);
% phi_target = asin(-ay_des / g_eff);
% theta_target = asin(ax_des / g_eff);

% Batasi sudut target (safety limit)
angle_max = deg2rad(25); % ±25 derajat
phi_target = max(-angle_max, min(angle_max, phi_target));
theta_target = max(-angle_max, min(angle_max, theta_target));

%% ===== INNER LOOP: Attitude Control =====

% --- Error Sudut ---
error_phi = phi_target - x_current(4);
error_theta = theta_target - x_current(5);
error_psi = x_target(6) - x_current(6);

% Wrap yaw error ke [-pi, pi]
error_psi = atan2(sin(error_psi), cos(error_psi));

% --- Error Kecepatan Sudut ---
% PERBAIKAN: Target angular rate = 0 (damping)
% Kita ingin drone stabil, jadi p, q, r harus → 0
error_p = 0 - x_current(10);  % Target p_dot = 0
error_q = 0 - x_current(11);  % Target q_dot = 0
error_r = 0 - x_current(12);  % Target r_dot = 0

% Atau bisa juga dihitung dari rate of change error sudut:
% phi_dot_target = Kp_phi * error_phi → simplified jadi p_target = 0

% --- PD Control untuk Attitude (NO INTEGRAL untuk hindari windup) ---
tau_phi = Kp(4) * error_phi + Kd(4) * error_p;       % Roll torque
tau_theta = Kp(5) * error_theta + Kd(5) * error_q;   % Pitch torque
tau_psi = Kp(6) * error_psi + Kd(6) * error_r;       % Yaw torque

%% ===== Motor Mixing: [F, tau_phi, tau_theta, tau_psi] → [u1, u2, u3, u4] =====
% Command vector
F_tau_cmd = [F_z_target; tau_phi; tau_theta; tau_psi];

% Inverse mixer (Mixer sudah didefinisikan di main)
u_pid = Mixer_Inv * F_tau_cmd;

%% ===== Motor Saturation =====
% Batasi setiap motor agar tetap dalam range fisik
u_min = params.u_min;
u_max = params.u_max;

for i = 1:4
    u_pid(i) = max(u_min, min(u_max, u_pid(i)));
end

 persistent call_count;
    if isempty(call_count), call_count = 0; end
    call_count = call_count + 1;
    
    if call_count <= 3  % Print 3 kali pertama saja
        fprintf('\n--- PID Internal (call #%d) ---\n', call_count);
        fprintf('error_pos(3)=%.3f, error_vel(3)=%.3f, PID_I(3)=%.3f\n', ...
                error_pos(3), error_vel(3), PID_I_state(3));
        fprintf('u_z = %.3f*%.3f + %.3f*%.3f + %.3f*%.3f = %.3f\n', ...
                Kp(3), error_pos(3), Ki(3), PID_I_state(3), Kd(3), error_vel(3), u_z);
        fprintf('F_z_target = %.3f * (%.3f + %.3f) = %.3f N\n', ...
                massa, gravitasi, u_z, F_z_target);
        fprintf('After saturation: F_z = %.3f N (limits: [%.2f, %.2f])\n', ...
                F_z_target, F_z_min, F_z_max);
    end

end