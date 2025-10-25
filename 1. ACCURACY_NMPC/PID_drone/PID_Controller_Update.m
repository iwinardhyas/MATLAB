function [des_acc, int_new] = PID_Controller_Update(params, pos_error, vel_error, int_old)
% PID_CONTROLLER_UPDATE: Menghitung akselerasi yang diinginkan dan mengupdate integral.
%   Input:
%     params    : Struct berisi Kp, Kd, Ki, Ki_sat, dt
%     pos_error : Error posisi [x;y;z]
%     vel_error : Error kecepatan [vx;vy;vz]
%     int_old   : Vektor integral lama (state)
%   Output:
%     des_acc   : Akselerasi yang diinginkan [ax;ay;az]
%     int_new   : Vektor integral yang telah diupdate

    % Ambil parameter
    Kp = params.Kp;
    Kd = params.Kd;
    Ki = params.Ki;
    Ki_sat = params.Ki_sat;
    dt = params.dt;

    % Pastikan input adalah vektor kolom
    pos_error = pos_error(:);
    vel_error = vel_error(:);
    int_old = int_old(:);
    
    % 1. Update Integral Controller
    int_new = int_old + pos_error .* dt;

    % 2. Prevent Windup (Anti-windup logic)
    
    % Cari indeks di mana integral melebihi batas saturasi
    over_mag = find(abs(int_new) > Ki_sat);
    
    if ~isempty(over_mag)
        for i = 1:length(over_mag)
            idx = over_mag(i);
            mag = abs(int_new(idx)); 
            
            % Mempertahankan arah (tanda) tetapi membatasi ke nilai saturasi
            int_new(idx) = sign(int_new(idx)) * Ki_sat(idx);
        end
    end
    
    % 3. Calculate controller input for desired acceleration
    % Operasi titik (.*) digunakan untuk perkalian element-wise (sesuai NumPy)
    des_acc = Kp .* pos_error + Ki .* int_new + Kd .* vel_error;
end