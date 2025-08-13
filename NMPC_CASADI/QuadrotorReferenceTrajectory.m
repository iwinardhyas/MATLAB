function [ xdesired ] = QuadrotorReferenceTrajectory( t )
    % Pastikan 't' adalah skalar saat fungsi dipanggil dalam loop simulasi.
    % Jika ingin mendukung vektor 't' untuk plotting terpisah, pastikan semua
    % operasi di dalam fungsi divetorisasi (menggunakan .*, ./, dll.).

    % --- Parameter Spiral (Sesuaikan untuk menyesuaikan gambar) ---
    radius = 1.5; % Radius spiral (perkiraan dari gambar)
    
    % Kecepatan angular untuk spiral (seberapa cepat berputar)
    % Sesuaikan nilai ini untuk mendapatkan jumlah putaran yang diinginkan
    % Misalnya, jika ingin satu putaran setiap 2*pi detik, maka freq = 1
    angular_freq = 0.5; % rad/detik (contoh: 0.5 berarti 1 putaran setiap 2*pi/0.5 = 12.56 detik)
    
    % Kecepatan naik vertikal (slope spiral)
    % Sesuaikan nilai ini agar Z naik sesuai keinginan.
    % Total Z dari 0 ke 4 dalam gambar. Jika total waktu simulasi 10 detik,
    % maka 4/10 = 0.4 m/s.
    z_rise_rate = 0.4; % m/detik (perkiraan)

    % Offset awal untuk posisi (jika spiral tidak dimulai dari (0,0,0))
    % Gambar menunjukkan Z dimulai dari sekitar 0, dan X,Y juga dekat 0.
    offset_x = 0; % Titik tengah spiral X
    offset_y = 0; % Titik tengah spiral Y
    initial_z = 0; % Ketinggian awal Z

    % --- Penerapan Scaling Factor (Untuk 'ramp-up' awal) ---
    % Ini penting agar drone tidak langsung melonjak ke trajectory.
    % Contoh: 'ramp-up' selama 2 detik pertama.
    ramp_duration = 2.0; % detik
    % Scaling factor akan linear dari 0 ke 1 selama ramp_duration
    current_scaling_factor = min(t / ramp_duration, 1.0); 

    % --- Hitungan Posisi (X, Y, Z) ---
    % X dan Y mengikuti lingkaran, Z naik linear
    x = offset_x + radius * cos(angular_freq * t) * current_scaling_factor;
    y = offset_y + radius * sin(angular_freq * t) * current_scaling_factor;
    z = initial_z + z_rise_rate * t;
    
    % Pastikan Z tidak melebihi batas atau tidak turun di bawah minimum yang wajar
    % (Jika Anda menginginkan Z maks 4, dan Z min 0)
    max_z = 4.0; % Ketinggian maksimum dari gambar
    z = min(max(z, initial_z), max_z); % Batasi Z antara initial_z dan max_z

    % --- Orientasi (phi, theta, psi) ---
    % Untuk trajectory pelacakan posisi, orientasi seringkali diatur nol
    % atau dihitung berdasarkan heading yang diinginkan.
    % Untuk spiral, yaw (psi) mungkin ingin mengikuti arah tangensial.
    % Kita biarkan nol dulu untuk menyederhanakan.
    phi = 0;
    theta = 0;
    psi = 0; % Atau bisa juga psi = angular_freq * t untuk membuat drone menghadap arah gerak

    % --- Kecepatan Linear (xdot, ydot, zdot) ---
    % Ini adalah bagian PENTING untuk NMPC yang melacak trajectory dinamis.
    % Kita perlu menghitung turunan waktu dari posisi X, Y, Z.
    % Turunan dari sin(at) adalah a*cos(at)
    % Turunan dari cos(at) adalah -a*sin(at)
    
    % Kecepatan jika scaling_factor = 1 (setelah ramp-up)
    % x_dot = -radius * angular_freq * sin(angular_freq * t);
    % y_dot = radius * angular_freq * cos(angular_freq * t);
    % z_dot = z_rise_rate;

    % Mengintegrasikan scaling_factor pada turunan: Ini lebih kompleks.
    % Jika t < ramp_duration: d/dt(current_scaling_factor * f(t))
    % = (d/dt current_scaling_factor) * f(t) + current_scaling_factor * (d/dt f(t))
    % d/dt (t/ramp_duration) = 1/ramp_duration
    if t < ramp_duration
        xdot = (1/ramp_duration) * (offset_x + radius * cos(angular_freq * t)) + ...
               current_scaling_factor * (-radius * angular_freq * sin(angular_freq * t));
        ydot = (1/ramp_duration) * (offset_y + radius * sin(angular_freq * t)) + ...
               current_scaling_factor * (radius * angular_freq * cos(angular_freq * t));
    else % Setelah ramp-up, scaling_factor konstan 1
        xdot = -radius * angular_freq * sin(angular_freq * t);
        ydot = radius * angular_freq * cos(angular_freq * t);
    end
    zdot = z_rise_rate; % Z dot konstan (selama belum mencapai max_z)
    
    % Kecepatan angular (phidot, thetadot, psidot) - biasanya nol untuk pelacakan posisi
    phidot = 0;
    thetadot = 0;
    psidot = 0;

    % Menggabungkan semua state yang diinginkan
    xdesired = [x;y;z;phi;theta;psi;xdot;ydot;zdot;phidot;thetadot;psidot];
end