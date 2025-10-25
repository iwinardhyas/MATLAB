function quad = Quadcopter_Step(quad, thrust_needed, tau_needed)
% QUADCOPTER_STEP: Melakukan satu langkah waktu simulasi dan mengupdate state quadcopter.
% EXACT MATCH dengan Python Quadcopter.py

    % Fungsi helper (cos, sin, tan)
    c = @cos; 
    s = @sin;
    t = @tan;

    % === 1. Motor Speed Calculation (des2speeds) ===
    % PERBAIKAN: EXACT match dengan Python logic
    
    % Needed torque on body - MULTIPLY BY INERTIA (seperti Python!)
    e1 = tau_needed(1) * quad.Ixx;  % Python: tau_des[0] * self.Ixx
    e2 = tau_needed(2) * quad.Iyy;  % Python: tau_des[1] * self.Iyy
    e3 = tau_needed(3) * quad.Izz;  % Python: tau_des[2] * self.Izz
    
    n = quad.num_motors;
    
    % Thrust desired converted into motor speeds (omega^2)
    % Python: weight_speed = thrust_des / (n*self.kt)
    weight_speed = thrust_needed / (n * quad.kt);
    
    % Thrust difference in each motor to achieve needed torque on body
    % IMPORTANT: motor_speeds di sini adalah omega^2 (rpm^2), BUKAN thrust!
    motor_speeds = zeros(n, 1);
    
    % Python logic EXACT:
    % motor_speeds.append(weight_speed - (e2/((n/2)*self.kt*self.L)) - (e3/(n*self.b_prop)))
    motor_speeds(1) = weight_speed - (e2/((n/2)*quad.kt*quad.L)) - (e3/(n*quad.b_prop));
    
    % motor_speeds.append(weight_speed - (e1/((n/2)*self.kt*self.L)) + (e3/(n*self.b_prop)))
    motor_speeds(2) = weight_speed - (e1/((n/2)*quad.kt*quad.L)) + (e3/(n*quad.b_prop));
    
    % motor_speeds.append(weight_speed + (e2/((n/2)*self.kt*self.L)) - (e3/(n*self.b_prop)))
    motor_speeds(3) = weight_speed + (e2/((n/2)*quad.kt*quad.L)) - (e3/(n*quad.b_prop));
    
    % motor_speeds.append(weight_speed + (e1/((n/2)*self.kt*self.L)) + (e3/(n*self.b_prop)))
    motor_speeds(4) = weight_speed + (e1/((n/2)*quad.kt*quad.L)) + (e3/(n*quad.b_prop));
    
    % Konversi motor speeds (omega^2) ke thrust (N)
    % Python: thrust_all = np.array(motor_speeds) * (self.kt)
    thrust_all = motor_speeds * quad.kt;
    
    % Ensure that desired thrust is within overall min and max
    % Python: over_max = np.argwhere(thrust_all > self.maxT)
    over_max = find(thrust_all > quad.maxT);
    under_min = find(thrust_all < quad.minT);
    
    if ~isempty(over_max)
        for i = 1:length(over_max)
            motor_speeds(over_max(i)) = quad.maxT / quad.kt;
        end
    end
    
    if ~isempty(under_min)
        for i = 1:length(under_min)
            motor_speeds(under_min(i)) = quad.minT / quad.kt;
        end
    end
    
    % Simpan motor speeds (omega^2)
    quad.speeds = motor_speeds;
    
    % === 2. Update Thrust Total ===
    % Python: self.thrust = self.kt * np.sum(self.speeds)
    quad.thrust = quad.kt * sum(quad.speeds);

    % === 3. Hitung Torque Tubuh (find_body_torque) ===
    % Python:
    % tau = np.array([
    %     (self.L * self.kt * (self.speeds[3] - self.speeds[1])),
    %     (self.L * self.kt * (self.speeds[2] - self.speeds[0])),
    %     (self.b_prop * (-self.speeds[0] + self.speeds[1] - self.speeds[2] + self.speeds[3]))
    % ])
    
    % PERBAIKAN: Gunakan speeds (omega^2), BUKAN thrust!
    quad.tau(1) = quad.L * quad.kt * (quad.speeds(4) - quad.speeds(2));  % Roll (phi)
    quad.tau(2) = quad.L * quad.kt * (quad.speeds(3) - quad.speeds(1));  % Pitch (theta)
    quad.tau(3) = quad.b_prop * (-quad.speeds(1) + quad.speeds(2) - quad.speeds(3) + quad.speeds(4));  % Yaw (psi)
    
    % === 4. Linear Acceleration (find_lin_acc) ===
    
    % Rotasi Body ke Inertial (R_B2I)
    % Python: body2inertial_rotation()
    c1 = c(quad.angle(1)); s1 = s(quad.angle(1));
    c2 = c(quad.angle(2)); s2 = s(quad.angle(2));
    c3 = c(quad.angle(3)); s3 = s(quad.angle(3));
    
    R_B2I = [c2*c3, c3*s1*s2 - c1*s3, s1*s3 + c1*s2*c3;
             c2*s3, c1*c3 + s1*s2*s3, c1*s3*s2 - c3*s1;
             -s2,   c2*s1,            c1*c2];
         
    R_I2B = R_B2I'; % Rotasi Inertial ke Body
    
    % Gaya Tubuh
    % Python: Thrust_body = np.array([0, 0, self.thrust])
    Thrust_body = [0; 0; quad.thrust];
    Thrust_inertial = R_B2I * Thrust_body;
    
    % Drag
    % Python: vel_bodyframe = np.matmul(R_I2B, self.vel)
    vel_bodyframe = R_I2B * quad.vel;
    drag_body = -quad.Cd * 0.5 * quad.density * quad.A_ref * (vel_bodyframe).^2;
    drag_inertial = R_B2I * drag_body;
    
    % Berat
    % Python: weight = self.mass * self.g (self.g = [0, 0, -gravity])
    weight = [0; 0; -quad.mass * quad.gravity];
    
    % Akselerasi Inertial
    % Python: acc_inertial = (Thrust_inertial + drag_inertial + weight) / self.mass
    acc_inertial = (Thrust_inertial + drag_inertial + weight) / quad.mass;
    quad.lin_acc = acc_inertial;
    
    % === 5. Angular Acceleration ===
    
    % Transformasi Euler Rate ke Angular Velocity Body Frame
    % Python: thetadot2omega()
    R_EtoOmega = [1, 0, -s(quad.angle(2));
                  0, c(quad.angle(1)), c(quad.angle(2))*s(quad.angle(1));
                  0, -s(quad.angle(1)), c(quad.angle(2))*c(quad.angle(1))];
    omega = R_EtoOmega * quad.ang_vel;
    
    % Angular Acceleration Body Frame (omega_dot)
    % Python: omega_dot = np.linalg.inv(self.I).dot(self.tau - np.cross(omega, np.matmul(self.I, omega)))
    I_omega = quad.I * omega;
    omega_dot = quad.I \ (quad.tau - cross(omega, I_omega));
    
    % Transformasi Angular Velocity Body Frame ke Euler Rate
    % Python: omegadot2Edot()
    R_OmegaToE = [1, s(quad.angle(1))*t(quad.angle(2)), c(quad.angle(1))*t(quad.angle(2));
                  0, c(quad.angle(1)), -s(quad.angle(1));
                  0, s(quad.angle(1))/c(quad.angle(2)), c(quad.angle(1))/c(quad.angle(2))];
    quad.ang_acc = R_OmegaToE * omega_dot;

    % === 6. Update States (Euler Integration) ===
    % Python: self.ang_vel += self.dt * self.ang_acc
    quad.ang_vel = quad.ang_vel + quad.dt * quad.ang_acc;
    quad.angle = quad.angle + quad.dt * quad.ang_vel;
    quad.vel = quad.vel + quad.dt * quad.lin_acc;
    quad.pos = quad.pos + quad.dt * quad.vel;
    quad.time = quad.time + quad.dt;
    
    % === 7. Batasan (Max Angle) - OPTIONAL (tidak ada di Python step()) ===
    % Python tidak punya ini di step(), tapi bisa ditambahkan untuk safety
    if isfield(quad, 'max_angle')
        mag_angle = norm(quad.angle);
        if mag_angle > quad.max_angle
            quad.angle = (quad.angle / mag_angle) * quad.max_angle;
            quad.ang_vel = quad.ang_vel * 0.5;  % Dampen velocity
        end
    end
end


% ===== CATATAN PENTING =====
% 
% 1. UNIT MOTOR SPEEDS:
%    - Python: motor_speeds dalam omega^2 (rpm^2)
%    - Thrust = kt * omega^2
%    - Torque = kt * L * (omega1^2 - omega2^2)
%
% 2. INERTIA MULTIPLICATION:
%    - Python MULTIPLY tau_des dengan Ixx, Iyy, Izz
%    - Ini karena input tau_des adalah ANGULAR ACCELERATION (rad/s^2)
%    - Output e1, e2, e3 adalah TORQUE (N·m)
%
% 3. MOTOR INDEXING:
%    - Python: 0-indexed (speeds[0], speeds[1], speeds[2], speeds[3])
%    - MATLAB: 1-indexed (speeds(1), speeds(2), speeds(3), speeds(4))
%
% 4. WEIGHT VECTOR:
%    - Python: self.g = np.array([0, 0, -gravity])
%    - MATLAB: weight = [0; 0; -mass*gravity]
%
% 5. DRAG CALCULATION:
%    - Element-wise square: (vel_bodyframe).^2 (MATLAB) vs
%    (vel_bodyframe)**2 (Python)