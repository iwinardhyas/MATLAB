%% QUADCOPTER_MAIN.M
% Terjemahan EXACT dari Quadcopter_main.py
% Fixed version dengan gain yang benar

% Bersihkan workspace dan command window
clear all; close all; clc;

% =================================
% 1. SETUP SIMULASI
% =================================

sim_start = 0;   % start time of simulation
sim_end = 15;    % end time of simulation in sec
dt = 0.01;       % step size in sec
time_index = sim_start:dt:sim_end;
N_sim = length(time_index);

% Initial reference state
r_ref = [0.; 0.; 3.]; % desired position [x, y, z] in inertial frame - meters

% Initial conditions
% Python: pos = [0.5, -0.5, 0.]
quad.pos = [0.5; -0.5; 0.]; % starting location [x, y, z] - meters
quad.vel = [0.; 0.; 0.];    % initial velocity - m/s
quad.angle = [0.; 0.; 0.];  % initial Euler angles [phi, theta, psi] - radians

% Initial random roll, pitch, and yaw rates (Perturbation)
% Python:
% deviation = 10
% random_set = np.array([random.random(), random.random(), random.random()])
% ang_vel = np.deg2rad(2* deviation * random_set - deviation)
deviation = 10; % magnitude of initial perturbation in deg/s
random_set = rand(3, 1); % MATLAB: rand(3,1) = Python: [random.random(), ...]
quad.ang_vel = deg2rad(2 * deviation * random_set - deviation);

% Untuk testing yang konsisten, bisa set manual:
% quad.ang_vel = [0; 0; 0]; % Jika ingin test tanpa perturbation

ang_vel_init = quad.ang_vel; % record for later display

% =================================
% 2. PARAMETER FISIK (Quadcopter.__init__)
% =================================
quad.mass = 0.506;      % total mass of the vehicle, kg
quad.gravity = 9.8;     % acceleration due to gravity, m/s^2 (Python: gravity = 9.8)
quad.num_motors = 4;
quad.kt = 1e-7;         % Thrust constant T=kt*omega^2
quad.b_prop = 1e-9;     % Torque constant (torque = b*omega^2)

quad.Ixx = 8.11858e-5;  % mass-moment of inertial about x-axis, kg-m^2
quad.Iyy = 8.11858e-5;
quad.Izz = 6.12233e-5;
quad.A_ref = 0.02;      % reference area for drag calcs, m^2 
quad.L = 0.2;           % length from body center to prop center, m
quad.Cd = 1;            % drag coefficient
quad.density = 1.225;   % air density, kg/m^3

quad.maxT = 16.5;       % max thrust from any single motor, N
quad.minT = 0.5;        % min thrust from any single motor, N 
quad.max_angle = pi/12; % radians, max angle allowed (Python: math.pi/12)
quad.thrust_total = quad.mass * quad.gravity; % Initial thrust (hover)

quad.I = diag([quad.Ixx, quad.Iyy, quad.Izz]); % Matriks Inersia
quad.g = [0; 0; -quad.gravity]; % Vektor Gravitasi (Python: [0, 0, -gravity])
quad.time = 0;
quad.dt = dt;

% Variabel State Referensi (diinisialisasi)
quad.pos_ref = r_ref;
quad.vel_ref = zeros(3, 1);
quad.lin_acc_ref = zeros(3, 1);
quad.angle_ref = zeros(3, 1);
quad.ang_vel_ref = zeros(3, 1);
quad.ang_acc_ref = zeros(3, 1);
quad.tau = zeros(3, 1);
quad.speeds = zeros(4, 1); % Motor speeds (omega^2)
quad.thrust_motors = zeros(4, 1); % Individual motor thrust

% =================================
% 3. SETUP KONTROLER PID
% =================================
% PERBAIKAN: EXACT match dengan Python gains!

% Gains for position controller
% Python: Kp_pos = [.95, .95, 15.]
Kp_pos = [0.95; 0.95; 15.0];  % ✅ DIPERBAIKI: 5.0 → 15.0
Kd_pos = [1.8; 1.8; 15.0];    % ✅ DIPERBAIKI: 10.0 → 15.0
Ki_pos = [0.2; 0.2; 1.0];     % ✅ DIPERBAIKI: 0.1 → 1.0
Ki_sat_pos = 1.1 * ones(3, 1); % ✅ DIPERBAIKI: 0.5 → 1.1

% Gains for angle controller
% Python: Kp_ang = [6.9, 6.9, 25.]
Kp_ang = [6.9; 6.9; 25.0]; % ✅ Sudah benar
Kd_ang = [3.7; 3.7; 9.0];  % ✅ Sudah benar
Ki_ang = [0.1; 0.1; 0.1];  % ✅ Sudah benar
Ki_sat_ang = 0.1 * ones(3, 1); % ✅ Sudah benar

% Structs untuk Controller (menggantikan class)
pos_cont_params.Kp = Kp_pos;
pos_cont_params.Kd = Kd_pos;
pos_cont_params.Ki = Ki_pos;
pos_cont_params.Ki_sat = Ki_sat_pos;
pos_cont_params.dt = dt;
pos_integral_state = [0.; 0.; 0.]; % Initial integral state

angle_cont_params.Kp = Kp_ang;
angle_cont_params.Kd = Kd_ang;
angle_cont_params.Ki = Ki_ang;
angle_cont_params.Ki_sat = Ki_sat_ang;
angle_cont_params.dt = dt;
angle_integral_state = [0.; 0.; 0.]; % Initial integral state


% =================================
% 4. INISIALISASI ARRAY LOGGING
% =================================

% Pre-allocate arrays (efisien di MATLAB)
position = zeros(3, N_sim);
velocity = zeros(3, N_sim);
angle = zeros(3, N_sim);
angle_vel = zeros(3, N_sim);
motor_thrust = zeros(4, N_sim);
body_torque = zeros(3, N_sim);
total_thrust = zeros(1, N_sim);
total_error = zeros(1, N_sim);


% =================================
% 5. SIMULASI LOOP
% =================================

fprintf('Starting simulation...\n');

for k = 1:N_sim

    % 5.1. HITUNG ERROR POSISI & PANGGIL KONTROLER POSISI
    % Python: pos_error = quadcopter.calc_pos_error(quadcopter.pos)
    pos_error = quad.pos_ref - quad.pos;
    vel_error = quad.vel_ref - quad.vel;
    
    % Panggil PID_Controller_Update untuk posisi
    % Python: des_acc = pos_controller.control_update(pos_error,vel_error)
    [des_acc, pos_integral_state] = PID_Controller_Update(pos_cont_params, pos_error, vel_error, pos_integral_state);
    
    % 5.2. MODIFIKASI Z-GAIN UNTUK HOVER
    % Python: des_acc[2] = (gravity + des_acc[2])/(math.cos(quadcopter.angle[0]) * math.cos(quadcopter.angle[1]))
    phi = quad.angle(1);   % Python: quadcopter.angle[0]
    theta = quad.angle(2); % Python: quadcopter.angle[1]
    
    des_acc(3) = (quad.gravity + des_acc(3)) / (cos(phi) * cos(theta));
    
    % Python: thrust_needed = quadcopter.mass * des_acc[2]
    thrust_needed = quad.mass * des_acc(3);

    % 5.3. HITUNG SUDUT YANG DIINGINKAN DARI AKSELERASI
    % Python: mag_acc = np.linalg.norm(des_acc)
    mag_acc = norm(des_acc);
    if mag_acc == 0
        mag_acc = 1; % Pencegahan bagi nol
    end
    
    % Python:
    % ang_des = [math.asin(-des_acc[1] / mag_acc / math.cos(quadcopter.angle[1])),
    %            math.asin(des_acc[0] / mag_acc),
    %            0]
    ang_des = [asin(-des_acc(2) / mag_acc / cos(theta));
               asin(des_acc(1) / mag_acc);
               0];

    % Python: Check if exceeds max angle
    mag_angle_des = norm(ang_des);
    if mag_angle_des > quad.max_angle
        ang_des = (ang_des / mag_angle_des) * quad.max_angle;
    end

    % 5.4. PANGGIL KONTROLER SUDUT
    % Python: quadcopter.angle_ref = ang_des
    quad.angle_ref = ang_des;
    ang_error = quad.angle_ref - quad.angle;
    ang_vel_error = quad.ang_vel_ref - quad.ang_vel;
    
    % Python: tau_needed = angle_controller.control_update(ang_error, ang_vel_error)
    [tau_needed, angle_integral_state] = PID_Controller_Update(angle_cont_params, ang_error, ang_vel_error, angle_integral_state);

    % 5.5. LANGKAHKAN SIMULASI
    % Python: quadcopter.des2speeds(thrust_needed, tau_needed)
    % Python: quadcopter.step()
    quad = Quadcopter_Step(quad, thrust_needed, tau_needed);

    % 5.6. LOGGING DATA
    % Python: position[0].append(quadcopter.pos[0])
    position(:, k) = quad.pos;
    velocity(:, k) = quad.vel;
    
    % Python: angle[0].append(np.rad2deg(quadcopter.angle[0]))
    angle(:, k) = rad2deg(quad.angle);
    angle_vel(:, k) = rad2deg(quad.ang_vel);
    
    % Python: motor_thrust[0].append(quadcopter.speeds[0]*quadcopter.kt)
    % Note: quad.thrust_motors sudah dalam Newton (dihitung di Quadcopter_Step)
    motor_thrust(:, k) = quad.thrust_motors;
    
    body_torque(:, k) = quad.tau;
    
    % Python: total_thrust.append(quadcopter.kt * np.sum(quadcopter.speeds))
    total_thrust(k) = quad.thrust_total;
    
    % Python: total_error.append(np.linalg.norm(r_error))
    total_error(k) = norm(quad.pos_ref - quad.pos);
    
    % Progress indicator (setiap 1 detik)
    if mod(k, round(1/dt)) == 0
        fprintf('  t = %.1f s, z = %.2f m, error = %.3f m\n', quad.time, quad.pos(3), total_error(k));
    end
end

fprintf('Simulation complete!\n\n');

% =================================
% 6. VISUALISASI HASIL (total_plot)
% =================================

% Python: write_init_ang_vel_to_screen()
fprintf('=== INITIAL CONDITIONS ===\n');
fprintf('Initial angular velocities (deg/s):\n');
fprintf('  Roll rate:  %.2f\n', rad2deg(ang_vel_init(1)));
fprintf('  Pitch rate: %.2f\n', rad2deg(ang_vel_init(2)));
fprintf('  Yaw rate:   %.2f\n', rad2deg(ang_vel_init(3)));
fprintf('Total magnitude of angular velocity: %.2f deg/s\n\n', norm(rad2deg(ang_vel_init)));

% Python: total_plot()
figure('Name', 'Quadcopter Simulation Results', 'Position', [100, 100, 1600, 800]);

% 3D Flight path
subplot(2, 4, 1);
plot3(position(1, :), position(2, :), position(3, :), 'b-', 'LineWidth', 1.5); hold on;
scatter3(r_ref(1), r_ref(2), r_ref(3), 100, 'r', 'o', 'filled');
scatter3(position(1,1), position(2,1), position(3,1), 100, 'g', 's', 'filled');
title('Flight Path');
xlabel('x (m)'); ylabel('y (m)'); zlabel('z (m)');
legend('Trajectory', 'Target', 'Start');
grid on; view(3);

% Lateral position plots
subplot(2, 4, 2);
plot(time_index, position(1, :), 'b', 'DisplayName', 'x'); hold on;
plot(time_index, position(2, :), 'r', 'DisplayName', 'y');
title('Lateral Postion');
xlabel('time (s)'); ylabel('position (m)');
legend('Location', 'best'); grid on;

% Vertical position plot
subplot(2, 4, 3);
plot(time_index, position(3, :), 'k', 'LineWidth', 1.5); hold on;
yline(r_ref(3), 'r--', 'LineWidth', 1.5, 'DisplayName', 'Target');
title('Vertical Position');
xlabel('time (s)'); ylabel('altitude (m)');
legend('Location', 'best'); grid on;

% Linear velocity plots
subplot(2, 4, 4);
plot(time_index, velocity(1, :), 'b', 'DisplayName', 'd(x)/dt'); hold on;
plot(time_index, velocity(2, :), 'r', 'DisplayName', 'd(y)/dt');
plot(time_index, velocity(3, :), 'k', 'DisplayName', 'd(z)/dt');
title('Linear Velocity');
xlabel('time (s)'); ylabel('velocity (m/s)');
legend('Location', 'best'); grid on;

% Motor thrust plots
subplot(2, 4, 5);
plot(time_index, motor_thrust(1, :), 'DisplayName', 'motor 1'); hold on;
plot(time_index, motor_thrust(2, :), 'DisplayName', 'motor 2');
plot(time_index, motor_thrust(3, :), 'DisplayName', 'motor 3');
plot(time_index, motor_thrust(4, :), 'DisplayName', 'motor 4');
yline(quad.maxT, 'r--', 'LineWidth', 1, 'DisplayName', 'Max');
yline(quad.minT, 'r--', 'LineWidth', 1, 'DisplayName', 'Min');
title('Motor Thrust');
xlabel('time (s)'); ylabel('Motor Thrust (N)');
legend('Location', 'best'); grid on;

% Body torque over time
subplot(2, 4, 6);
plot(time_index, body_torque(1, :), 'b', 'DisplayName', 'x (Roll)'); hold on;
plot(time_index, body_torque(2, :), 'r', 'DisplayName', 'y (Pitch)');
plot(time_index, body_torque(3, :), 'k', 'DisplayName', 'z (Yaw)');
title('Body Torque');
xlabel('time (s)'); ylabel('torque (N·m)');
legend('Location', 'best'); grid on;

% Angles over time
subplot(2, 4, 7);
plot(time_index, angle(1, :), 'b', 'DisplayName', 'phi (Roll)'); hold on;
plot(time_index, angle(2, :), 'r', 'DisplayName', 'theta (Pitch)');
plot(time_index, angle(3, :), 'k', 'DisplayName', 'psi (Yaw)');
yline(rad2deg(quad.max_angle), 'r--', 'DisplayName', 'Max angle');
yline(-rad2deg(quad.max_angle), 'r--');
title('Euler Angles');
xlabel('time (s)'); ylabel('angle (deg)');
legend('Location', 'best'); grid on;

% Angular velocity over time
subplot(2, 4, 8);
plot(time_index, angle_vel(1, :), 'b', 'DisplayName', 'd(phi)/dt'); hold on;
plot(time_index, angle_vel(2, :), 'r', 'DisplayName', 'd(theta)/dt');
plot(time_index, angle_vel(3, :), 'k', 'DisplayName', 'd(psi)/dt');
title('Angular Velocity');
xlabel('time (s)'); ylabel('angular velocity (deg/s)');
legend('Location', 'best'); grid on;

% Print final statistics
fprintf('=== FINAL STATISTICS ===\n');
fprintf('Final position: [%.3f, %.3f, %.3f] m\n', quad.pos);
fprintf('Target position: [%.3f, %.3f, %.3f] m\n', r_ref);
fprintf('Final error: %.4f m\n', total_error(end));
fprintf('Average error: %.4f m\n', mean(total_error));
fprintf('Max angle reached: %.2f deg\n', max(abs(angle(:))));