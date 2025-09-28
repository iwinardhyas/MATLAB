function [ xdesired ] = QuadrotorReferenceTrajectory4( t )

% FORCE debug mode - pastikan ini yang dipanggil
trajectory_type = 'figure_eight';

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
    case 'aggressive_but_smooth'
    % Lintasan agresif tapi smooth secara dinamis
    amplitude_y = 1.0;         % m
    frequency_y = 0.5;         % Hz (agak cepat)
    forward_speed = 2.0;       % m/s (cukup cepat)
    altitude = 2.0;            % m
    g = 9.81;
    persistent psi_prev

    % Posisi
    x = forward_speed * t;
    y = amplitude_y * sin(2 * pi * frequency_y * t);
    z = altitude;

    % Kecepatan (1st derivative)
    omega = 2 * pi * frequency_y;
    xdot = forward_speed;
    ydot = amplitude_y * omega * cos(omega * t);
    zdot = 0;

    % Percepatan lateral (2nd derivative)
    ay = -amplitude_y * omega^2 * sin(omega * t);

    % Roll angle dibatasi ±20° untuk banking
    phi = atan2(ay, g);
    phi = max(min(phi, deg2rad(15)), deg2rad(-15));

    % Estimasi heading agar menghadap arah kecepatan (smooth yaw)
    psi_unfiltered  = atan2(ydot, xdot);
    % Inisialisasi psi_prev saat pertama kali fungsi dipanggil
    if isempty(psi_prev)
        psi_prev = psi_unfiltered;
    end
%     psi = wrapToPi(psi);
    alpha = 0.2;
    psi = alpha * psi_unfiltered + (1-alpha) * psi_prev;  % gunakan persistent variable

    % Estimasi rate orientasi
    phidot = 0;  % atau bisa dikira dari d(phi)/dt numerik
    thetadot = 0;
    psidot = (omega * -amplitude_y * omega * sin(omega * t)) / ...
             (1 + (ydot/xdot)^2); % d(atan2)/dt approx

    % Lengkapi
    theta = 0;

    xdesired = [x; y; z; phi; theta; psi;
                xdot; ydot; zdot;
                phidot; thetadot; psidot];
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
%         psi = atan2(ydot, xdot); % Menghadap arah gerakan
        psi = 0;
        
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
        phi = max(min(phi, deg2rad(25)), deg2rad(-25)); % Batasi ke ±15°

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
        psi = 0;

        phidot = 0; thetadot = 0; psidot = angular_speed;

        xdesired = [x;y;z;phi;theta;psi;xdot;ydot;zdot;phidot;thetadot;psidot];
    case 'figure_eight'
        % Definisi Trajectory Sinusoidal (Figure Eight)
        % Xd = [0.5 * sin(0.3*t); 0.3 * sin(0.6*t); 0.4]

        % --- PARAMETER ---
        g = 9.81;
        amplitude_x = 0.5;
        frequency_x = 0.3; % rad/s
        amplitude_y = 0.3;
        frequency_y = 0.6; % rad/s
        altitude_z = 0.4;

        % --- POSISI (x, y, z) ---
        x = amplitude_x * sin(frequency_x * t);
        y = amplitude_y * sin(frequency_y * t);
        z = altitude_z;

        % --- KECEPATAN (xdot, ydot, zdot) ---
        xdot = amplitude_x * frequency_x * cos(frequency_x * t);
        ydot = amplitude_y * frequency_y * cos(frequency_y * t);
        zdot = 0;

        % --- PERCEPATAN (ax, ay) ---
        % Diperlukan untuk menghitung sudut kemiringan (Roll dan Pitch)
        ax = -amplitude_x * frequency_x^2 * sin(frequency_x * t);
        ay = -amplitude_y * frequency_y^2 * sin(frequency_y * t);
        az = 0; % Ketinggian konstan

        % --- ORIENTASI (phi, theta, psi) ---

        % Hitung Roll (phi) dan Pitch (theta) yang dibutuhkan untuk menghasilkan ax dan ay
        % F_thrust_x = m * ax = F_total * (R_b_i)_x_3
        % F_thrust_y = m * ay = F_total * (R_b_i)_y_3
        % F_thrust_z = m * (az + g) = F_total * (R_b_i)_z_3

        % Jika kita mengasumsikan psi = 0 (menghadap ke depan konstan):
        % R_b_i * [0; 0; F_total] = [F_total*sin(theta); -F_total*cos(theta)*sin(phi); F_total*cos(theta)*cos(phi)]
        % Kita perlu sudut roll dan pitch untuk menghasilkan ax dan ay.

        % Pitch (theta) dikontrol oleh Percepatan Sumbu X (ax)
        theta = asin(ax / g);

        % Roll (phi) dikontrol oleh Percepatan Sumbu Y (ay), dikoreksi oleh theta
        phi = asin(-ay / (g * cos(theta)));

        % Kita akan asumsikan yaw (psi) tetap nol (hanya bergerak maju/mundur)
        psi = 0; 

        % --- LAJU SUDUT (phidot, thetadot, psidot) ---
        % NMPC yang baik seharusnya dapat menghitung turunan ini sendiri
        % Tetapi untuk referensi, kita asumsikan laju sudut adalah nol (kondisi ideal)
        % Anda mungkin perlu menurunkan turunan phi dan theta jika NMPC Anda membutuhkan ini secara eksplisit.
        phidot = 0; 
        thetadot = 0;
        psidot = 0; 

        % Gabungkan semua variabel ke dalam vektor Xdesired
        xdesired = [x; y; z; phi; theta; psi; xdot; ydot; zdot; phidot; thetadot; psidot];

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