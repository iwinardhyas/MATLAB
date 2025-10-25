% Asumsi: objek quadcopter dan pos_controller sudah dibuat sebelum skrip ini.
% Contoh (sesuaikan):
% dt = 0.01; T = 10;
% quadcopter = Quadcopter([0],[0],[0],[0], dt);
% pos_controller = PID_Controller([Kp_x Kp_y Kp_z], [Kd_x Kd_y Kd_z], [Ki_x Ki_y Ki_z], [sat_x sat_y sat_z], dt);
% quadcopter.pos_ref = [x_ref y_ref z_ref];
dt = 0.01; T = 10;
pos = 0; vel = 0;
ang = 0; ang_vel = 0;
r_ref = 0;
% Waktu simulasi
% Pastikan dt dan T didefinisikan
time_index = 0:quadcopter.dt:T;
N = numel(time_index);

gravity = 9.8; % m/s^2 
Quadcopter_main.py

% Prealokasi hasil
position_total = zeros(1,N);

position = cell(1,3); velocity = cell(1,3);
angle = cell(1,3); angle_vel = cell(1,3);
for i = 1:3
position{i} = zeros(1,N);
velocity{i} = zeros(1,N);
angle{i} = zeros(1,N);
angle_vel{i} = zeros(1,N);
end

motor_thrust = cell(1,4);
for m = 1:4
motor_thrust{m} = zeros(1,N);
end

body_torque = cell(1,3);
for i = 1:3
body_torque{i} = zeros(1,N);
end

total_thrust = zeros(1,N);
total_error = zeros(1,N);

% Simulasi 
Quadcopter_main.py

for k = 1:N
% Error posisi dan kecepatan + kontroler posisi 
Quadcopter_main.py

pos_error = quadcopter.calc_pos_error(quadcopter.pos);
vel_error = quadcopter.calc_vel_error(quadcopter.vel);
des_acc = pos_controller.control_update(pos_error, vel_error);

% Koreksi sumbu z untuk hover <source_id data="3" title="Quadcopter_main.py" />
des_acc(3) = (gravity + des_acc(3)) / (cos(quadcopter.angle(1)) * cos(quadcopter.angle(2)));

% Thrust yang dibutuhkan <source_id data="3" title="Quadcopter_main.py" />
thrust_needed = quadcopter.mass * des_acc(3);

% Torsi yang dibutuhkan (placeholder: ganti dengan kontroler sudut Anda)
tau_needed = zeros(3,1);

% Kecepatan motor untuk mencapai thrust dan torsi target <source_id data="3" title="Quadcopter_main.py" />
quadcopter.des2speeds(thrust_needed, tau_needed);

% Integrasi langkah waktu dan update state <source_id data="3" title="Quadcopter_main.py" />
quadcopter.step();

% Rekam hasil untuk plotting/analisis <source_id data="3" title="Quadcopter_main.py" />
position_total(k) = norm(quadcopter.pos);

position{1}(k) = quadcopter.pos(1);
position{2}(k) = quadcopter.pos(2);
position{3}(k) = quadcopter.pos(3);

velocity{1}(k) = quadcopter.vel(1);
velocity{2}(k) = quadcopter.vel(2);
velocity{3}(k) = quadcopter.vel(3);

angle{1}(k) = rad2deg(quadcopter.angle(1));
angle{2}(k) = rad2deg(quadcopter.angle(2));
angle{3}(k) = rad2deg(quadcopter.angle(3));

angle_vel{1}(k) = rad2deg(quadcopter.ang_vel(1));
angle_vel{2}(k) = rad2deg(quadcopter.ang_vel(2));
angle_vel{3}(k) = rad2deg(quadcopter.ang_vel(3));

motor_thrust{1}(k) = quadcopter.speeds(1) * quadcopter.kt;
motor_thrust{2}(k) = quadcopter.speeds(2) * quadcopter.kt;
motor_thrust{3}(k) = quadcopter.speeds(3) * quadcopter.kt;
motor_thrust{4}(k) = quadcopter.speeds(4) * quadcopter.kt;

body_torque{1}(k) = quadcopter.tau(1);
body_torque{2}(k) = quadcopter.tau(2);
body_torque{3}(k) = quadcopter.tau(3);

total_thrust(k) = quadcopter.kt * sum(quadcopter.speeds); % <source_id data="3" title="Quadcopter_main.py" />

r_error = quadcopter.pos_ref - quadcopter.pos;
total_error(k) = norm(r_error); % <source_id data="3" title="Quadcopter_main.py" />
end