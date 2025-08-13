% function [ xdesired ] = QuadrotorReferenceTrajectory( t, nominal_mass )
%     % QuadrotorReferenceTrajectory - Menghasilkan referensi trajectory yang halus untuk drone.
%     % Menggunakan kurva Bezier kubik untuk transisi yang mulus.
%     % Memungkinkan observasi prediksi payload UKF dengan tetap menjaga stabilitas NMPC.
% 
%     % --- Parameter Trajectory ---
%     initial_x = 0;
%     initial_y = 0;
%     initial_z = 0.5; % Ketinggian awal
% 
%     target_x = 1.0;    % Posisi X target setelah transisi
%     target_y = 1.0;    % Posisi Y target setelah transisi
%     target_z = 2.0;    % Posisi Z target setelah transisi
% 
%     % Waktu transisi
%     start_transition_time = 2.0; % Waktu (detik) ketika transisi posisi dimulai
%     transition_duration = 3.0;   % Durasi (detik) transisi dari posisi awal ke target
% 
%     end_transition_time = start_transition_time + transition_duration;
% 
%     % Posisi dan Kecepatan Default (sebelum transisi atau setelah transisi selesai)
%     x = initial_x;
%     y = initial_y;
%     z = initial_z;
%     xdot = 0;
%     ydot = 0;
%     zdot = 0;
% 
%     % --- Logika Transisi Halus ---
%     if t >= start_transition_time && t < end_transition_time
%         % Hitung parameter waktu normalisasi t_norm dalam [0, 1]
%         t_norm = (t - start_transition_time) / transition_duration;
% 
%         % Fungsi Smoothstep (Bezier Cubic): f(t_norm) = 3*t_norm^2 - 2*t_norm^3
%         % Fungsi ini memastikan turunan (kecepatan) juga nol di awal dan akhir transisi.
%         smooth_factor = 3 * t_norm^2 - 2 * t_norm^3;
% 
%         % Turunan dari smooth_factor terhadap t_norm: f'(t_norm) = 6*t_norm - 6*t_norm^2
%         % Untuk mendapatkan kecepatan sebenarnya, kita perlu mengalikan dengan (delta_pos / transition_duration)
%         smooth_factor_dot = (6 * t_norm - 6 * t_norm^2);
% 
%         % Posisi
%         x = initial_x + (target_x - initial_x) * smooth_factor;
%         y = initial_y + (target_y - initial_y) * smooth_factor;
%         z = initial_z + (target_z - initial_z) * smooth_factor;
% 
%         % Kecepatan (derivatif dari posisi terhadap waktu t)
%         xdot = (target_x - initial_x) * smooth_factor_dot / transition_duration;
%         ydot = (target_y - initial_y) * smooth_factor_dot / transition_duration;
%         zdot = (target_z - initial_z) * smooth_factor_dot / transition_duration;
% 
%     elseif t >= end_transition_time
%         % Setelah transisi selesai, tetap di posisi target
%         x = target_x;
%         y = target_y;
%         z = target_z;
%         % Kecepatan sudah nol di akhir transisi berkat smoothstep
%         xdot = 0;
%         ydot = 0;
%         zdot = 0;
%     end
%     
%     % --- Perubahan Payload ---
%     % Skenario perubahan payload tetap pada waktu yang Anda inginkan
%     % Misalkan perubahan payload terjadi pada t = 5.0 detik seperti yang Anda definisikan di main_payload_UKF_NMPC.m
%     % Trajectory referensi tidak secara langsung mengubah massa, melainkan NMPC akan bereaksi terhadap estimasi massa UKF.
%     % Pastikan time_of_payload_change di main_payload_UKF_NMPC.m lebih besar dari end_transition_time
%     % agar NMPC sempat stabil di posisi target sebelum massa berubah.
% 
%     % Orientasi (nol)
%     phi = 0;
%     theta = 0;
%     psi = 0;
% 
%     % Kecepatan Sudut (nol)
%     phidot = 0;
%     thetadot = 0;
%     psidot = 0;
% 
%     % Gabungkan semua state yang diinginkan
%     xdesired = [x;y;z;phi;theta;psi;xdot;ydot;zdot;phidot;thetadot;psidot];
% end

function [ xdesired ] = QuadrotorReferenceTrajectory4( t, nominal_mass )

% FORCE debug mode - pastikan ini yang dipanggil
trajectory_type = 'debug_hover';

% Inisialisasi output
xdesired = zeros(12, 1);

% DEBUG: print untuk memastikan function dipanggil
persistent call_count;
if isempty(call_count)
    call_count = 0;
end
call_count = call_count + 1;

% Only print first few calls to avoid spam
if call_count <= 5
    fprintf('DEBUG: Trajectory called with t=%.3f, mode=%s\n', t, trajectory_type);
end

switch trajectory_type
    case 'debug_hover'
        % Trajectory paling sederhana - naik perlahan lalu hover
        target_altitude = 2.0;
        rise_time = 4.0; % 4 detik untuk naik
        
        if t <= rise_time
            % Naik perlahan dengan smooth acceleration
            progress = t / rise_time;
            % Smooth S-curve
            smooth_progress = 3*progress^2 - 2*progress^3;
            z = target_altitude * smooth_progress;
            zdot = target_altitude * (6*progress - 6*progress^2) / rise_time;
        else
            % Hover di ketinggian target
            z = target_altitude;
            zdot = 0;
        end
        
        % Posisi horizontal tetap nol
        x = 0; y = 0;
        xdot = 0; ydot = 0;
        
        % Orientasi level (no rotation)
        phi = 0; theta = 0; psi = 0;
        phidot = 0; thetadot = 0; psidot = 0;
        
        xdesired = [x; y; z; phi; theta; psi; xdot; ydot; zdot; phidot; thetadot; psidot];
        
        % DEBUG: print trajectory values
        if call_count <= 5
            fprintf('  -> Returning: x=%.3f, y=%.3f, z=%.3f, zdot=%.3f\n', x, y, z, zdot);
        end
        
    case 'debug_circle'
        % Trajectory lingkaran horizontal sederhana
        radius = 1.0;     % meter
        omega = 0.5;      % rad/s (lambat)
        altitude = 2.0;   % meter
        
        x = radius * cos(omega * t);
        y = radius * sin(omega * t);
        z = altitude;
        
        xdot = -radius * omega * sin(omega * t);
        ydot = radius * omega * cos(omega * t);
        zdot = 0;
        
        % Banking angle untuk belokan
        a_centripetal = radius * omega^2;
        phi = atan(a_centripetal / 9.81);
        theta = 0;
        psi = atan2(ydot, xdot); % Menghadap arah gerakan
        
        phidot = 0; thetadot = 0; psidot = omega;
        
        xdesired = [x; y; z; phi; theta; psi; xdot; ydot; zdot; phidot; thetadot; psidot];
    
    case 'fast_sine'
        % Parameter yang lebih konservatif
        amplitude_y = 1.0;         % Kurangi amplitude
        frequency_y = 0.3;         % Kurangi frequency
        forward_speed = 1.5;       % Kurangi speed
        altitude = 2.0;            % Kurangi altitude
        g = 9.81;

        % Posisi
        x = forward_speed * t;
        y = amplitude_y * sin(2 * pi * frequency_y * t);
        z = altitude;

        % Kecepatan
        xdot = forward_speed;
        ydot = amplitude_y * 2 * pi * frequency_y * cos(2 * pi * frequency_y * t);
        zdot = 0;

        % Percepatan lateral untuk roll angle
        ay = diff2(amplitude_y, frequency_y, t);
        phi = atan(ay / g);
        phi = max(min(phi, deg2rad(15)), deg2rad(-15)); % Batasi ke ±15°

        theta = 0; psi = 0;
        phidot = 0; thetadot = 0; psidot = 0;

        xdesired = [x; y; z; phi; theta; psi; xdot; ydot; zdot; phidot; thetadot; psidot];

    case 'helical'
        % Parameter yang lebih konservatif
        radius = 1.5;       % Kurangi radius
        angular_speed = 0.5; % Kurangi kecepatan angular
        vertical_speed = 0.3; % Kurangi kecepatan vertikal
        start_altitude = 1.0; % Mulai lebih rendah

        x = radius * cos(angular_speed * t);
        y = radius * sin(angular_speed * t);
        z = start_altitude + vertical_speed * t;

        xdot = -radius * angular_speed * sin(angular_speed * t);
        ydot = radius * angular_speed * cos(angular_speed * t);
        zdot = vertical_speed;

        % Banking angle
        a_centripetal = radius * angular_speed^2;
        phi = atan(a_centripetal / 9.81);
        theta = 0;
        psi = atan2(ydot, xdot);

        phidot = 0; thetadot = 0; psidot = angular_speed;

        xdesired = [x;y;z;phi;theta;psi;xdot;ydot;zdot;phidot;thetadot;psidot];

    otherwise
        % Default: hovering di ketinggian 1m
        xdesired = [0;0;1;0;0;0;0;0;0;0;0;0];
end

end

% Fungsi percepatan lateral
function a = diff2(A, f, t)
    omega = 2 * pi * f;
    a = -A * omega^2 * sin(omega * t);
end