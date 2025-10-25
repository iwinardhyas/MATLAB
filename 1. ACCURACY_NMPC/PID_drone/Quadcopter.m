classdef Quadcopter < handle
properties
    mass
    gravity
    num_motors
    kt
    pos
    vel
    angle
    ang_vel
    lin_acc
    ang_acc
    dt
    time
    ang_vel_ref
end
methods
function self = Quadcopter(pos, vel, angle, ang_vel, dt)
    % Konstanta fisik sesuai sumber
    self.mass = 0.506; % kg 
    Quadcopter.py

    self.gravity = 9.8; % m/s^2 
    Quadcopter.py

    self.num_motors = 4; % 
    Quadcopter.py

    self.kt = 1e-7; % N/(rpm)^2 
    Quadcopter.py

    % State awal
    self.pos = pos(:).';
    self.vel = vel(:).';
    self.angle = angle(:).';
    self.ang_vel = ang_vel(:).';
    self.lin_acc = zeros(3,1); % <source_id data="2" title="Quadcopter.py" />
    self.ang_acc = zeros(3,1);
    self.dt = dt;
    self.time = 0;
    self.ang_vel_ref = zeros(3,1);
end

function ang_vel_error = calc_ang_vel_error(self, ang_vel)
    % Error kecepatan sudut: ref - current <source_id data="2" title="Quadcopter.py" />
    ang_vel_error = self.ang_vel_ref - ang_vel;
end

function R = body2inertial_rotation(self)
    % Rotasi Euler body->inertial (roll, pitch, yaw) <source_id data="2" title="Quadcopter.py" />
    c1 = cos(self.angle(1)); s1 = sin(self.angle(1));
    c2 = cos(self.angle(2)); s2 = sin(self.angle(2));
    c3 = cos(self.angle(3)); s3 = sin(self.angle(3));

    R = [ c2*c3, c3*s1*s2 - c1*s3, s1*s3 + c1*s2*c3; ...
    c2*s3, c1*c3 + s1*s2*s3, c1*s3*s2 - c3*s1; ...
    -s2, c2*s1, c1*c2 ];
end

function step(self)
    % Dinamika sudut dan update state per langkah waktu <source_id data="2" title="Quadcopter.py" />

    % Torsi badan dari diferensial thrust motor
    self.find_body_torque(); % TODO: isi agar setara dengan Python

    % Percepatan sudut dalam frame inersia
    omega = self.thetadot2omega(); % TODO: isi sesuai Python
    omega_dot = self.find_omegadot(omega); % TODO: isi sesuai Python

    % Ubah ke percepatan sudut di body/inersia sesuai implementasi Python
    self.omegadot2Edot(omega_dot); % TODO: isi sesuai Python

    % Update state (integrasi Euler) <source_id data="2" title="Quadcopter.py" />
    self.ang_vel = self.ang_vel + self.dt * self.ang_acc;
    self.angle = self.angle + self.dt * self.ang_vel;
    self.vel = self.vel + self.dt * self.lin_acc;
    self.pos = self.pos + self.dt * self.vel;
    self.time = self.time + self.dt;
end

% ====== Stub yang perlu diisi dari kode Python aslinya ======
function find_body_torque(self)
% TODO: hitung torsi badan dari diferensial thrust motor
% Harus mengisi variabel/efek yang digunakan downstream.
end

function omega = thetadot2omega(self)
% TODO: konversi laju Euler ke laju body/inersia sesuai Python
omega = self.ang_vel; % placeholder agar tidak error
end

function omega_dot = find_omegadot(self, omega)
    % TODO: dinamika rotasi untuk mendapatkan omega_dot
    omega_dot = zeros(3,1); % placeholder
end

function omegadot2Edot(self, omega_dot)
    % TODO: konversi omega_dot ke ang_acc sesuai kerangka yang dipakai
    self.ang_acc = omega_dot; % placeholder
end
end
end