function [ xdesired ] = QuadrotorReferenceTrajectory( t )
    % --- Parameter Spiral (Sesuaikan untuk menyesuaikan gambar) ---
    radius = 1.5;         % Radius spiral (perkiraan dari gambar)
    angular_freq = 0.5;   % rad/detik (kecepatan putar)
    z_rise_rate = 0.4;    % m/detik (kecepatan naik Z)

    offset_x = 0;         % Titik tengah spiral X
    offset_y = 0;         % Titik tengah spiral Y
    initial_z = 0;        % Ketinggian awal Z

    max_z = 4.0;          % Ketinggian maksimum dari gambar

    % --- Penerapan Scaling Factor (Untuk 'ramp-up' awal) ---
    ramp_duration = 2.0; % detik
    current_scaling_factor = min(t / ramp_duration, 1.0); 

    % --- Hitungan Posisi (X, Y, Z) ---
    x = offset_x + radius * cos(angular_freq * t) * current_scaling_factor;
    y = offset_y + radius * sin(angular_freq * t) * current_scaling_factor;
    
    z_unclamped = initial_z + z_rise_rate * t; % Hitung Z tanpa batasan dulu
    z = min(max(z_unclamped, initial_z), max_z); % Batasi Z antara initial_z dan max_z

    % --- Orientasi (phi, theta, psi) ---
    phi = 0;
    theta = 0;
    psi = 0; % Untuk spiral horizontal, yaw mungkin ingin mengikuti arah tangensial: psi = angular_freq * t + atan2(dy_offset, dx_offset);

    % --- Kecepatan Linear (xdot, ydot, zdot) ---
    % Default: Setelah ramp-up (current_scaling_factor = 1)
    xdot = -radius * angular_freq * sin(angular_freq * t);
    ydot = radius * angular_freq * cos(angular_freq * t);
    
    % Perhitungan zdot
    if z_unclamped >= max_z % Jika Z sudah mencapai atau melebihi batas atas
        zdot = 0; % Berhenti naik
    else
        zdot = z_rise_rate; % Terus naik
    end

    % Penyesuaian kecepatan selama fase ramp-up
    if t < ramp_duration
        % Turunan dari (t/ramp_duration) * F(t) adalah:
        % (1/ramp_duration) * F(t) + (t/ramp_duration) * F_dot(t)
        
        % Bagian F(t)
        F_x_t = radius * cos(angular_freq * t);
        F_y_t = radius * sin(angular_freq * t);

        % Bagian F_dot(t)
        F_dot_x_t = -radius * angular_freq * sin(angular_freq * t);
        F_dot_y_t = radius * angular_freq * cos(angular_freq * t);

        xdot = (1/ramp_duration) * (offset_x + F_x_t) + (t/ramp_duration) * F_dot_x_t;
        ydot = (1/ramp_duration) * (offset_y + F_y_t) + (t/ramp_duration) * F_dot_y_t;
        
        % Catatan: Jika offset_x/y juga dikalikan scaling_factor, turunannya seperti di atas.
        % Jika offset_x/y tidak diskalakan (tetap konstan), maka tidak ada (1/ramp_duration) * offset_x/y
        % Asumsi dari definisi x,y Anda sebelumnya: offset_x/y tidak diskalakan di *dalam* sin/cos,
        % melainkan ditambahkan setelah perkalian scaling_factor.
        % Jadi, definisi yang lebih tepat untuk x,y Anda sebelumnya adalah:
        % x = scaling_factor .* (6 .* sin(t/3) + 6) * (5/6);
        % y = scaling_factor .* (-6 .* sin(t/3).*cos(t/3) + 6) * (5/6);
        % Ini berarti offset 6 * (5/6) = 5 juga diskalakan.
        % Mari kita ikuti yang di atas (offset_x/y ditambahkan setelah diskala).
        % Jika offset Anda *tidak* diskalakan:
        % x = radius * cos(...) * scaling_factor + offset_x
        % xdot = radius * cos(...) / ramp_duration + radius * sin(...) * omega * scaling_factor
        % Ini tergantung bagaimana Anda mendefinisikan "offset" Anda.
        % Saya telah menggunakan rumus yang benar untuk x = offset + factor * func(t)
        % Jadi ini harusnya akurat.
    end

    % Kecepatan angular (phidot, thetadot, psidot)
    phidot = 0;
    thetadot = 0;
    psidot = 0;

    % Menggabungkan semua state yang diinginkan
    xdesired = [x;y;z;phi;theta;psi;xdot;ydot;zdot;phidot;thetadot;psidot];
end