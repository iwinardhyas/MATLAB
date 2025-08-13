function [ xdesired ] = QuadrotorReferenceTrajectory( t, nominal_mass )
    % QuadrotorReferenceTrajectory - Menghasilkan referensi trajectory yang halus untuk drone.
    % Menggunakan kurva Bezier kubik untuk transisi yang mulus.
    % Memungkinkan observasi prediksi payload UKF dengan tetap menjaga stabilitas NMPC.

    % --- Parameter Trajectory ---
    initial_x = 0;
    initial_y = 0;
    initial_z = 0.5; % Ketinggian awal

    target_x = 1.0;    % Posisi X target setelah transisi
    target_y = 1.0;    % Posisi Y target setelah transisi
    target_z = 2.0;    % Posisi Z target setelah transisi

    % Waktu transisi
    start_transition_time = 2.0; % Waktu (detik) ketika transisi posisi dimulai
    transition_duration = 3.0;   % Durasi (detik) transisi dari posisi awal ke target

    end_transition_time = start_transition_time + transition_duration;

    % Posisi dan Kecepatan Default (sebelum transisi atau setelah transisi selesai)
    x = initial_x;
    y = initial_y;
    z = initial_z;
    xdot = 0;
    ydot = 0;
    zdot = 0;

    % --- Logika Transisi Halus ---
    if t >= start_transition_time && t < end_transition_time
        % Hitung parameter waktu normalisasi t_norm dalam [0, 1]
        t_norm = (t - start_transition_time) / transition_duration;

        % Fungsi Smoothstep (Bezier Cubic): f(t_norm) = 3*t_norm^2 - 2*t_norm^3
        % Fungsi ini memastikan turunan (kecepatan) juga nol di awal dan akhir transisi.
        smooth_factor = 3 * t_norm^2 - 2 * t_norm^3;

        % Turunan dari smooth_factor terhadap t_norm: f'(t_norm) = 6*t_norm - 6*t_norm^2
        % Untuk mendapatkan kecepatan sebenarnya, kita perlu mengalikan dengan (delta_pos / transition_duration)
        smooth_factor_dot = (6 * t_norm - 6 * t_norm^2);

        % Posisi
        x = initial_x + (target_x - initial_x) * smooth_factor;
        y = initial_y + (target_y - initial_y) * smooth_factor;
        z = initial_z + (target_z - initial_z) * smooth_factor;

        % Kecepatan (derivatif dari posisi terhadap waktu t)
        xdot = (target_x - initial_x) * smooth_factor_dot / transition_duration;
        ydot = (target_y - initial_y) * smooth_factor_dot / transition_duration;
        zdot = (target_z - initial_z) * smooth_factor_dot / transition_duration;

    elseif t >= end_transition_time
        % Setelah transisi selesai, tetap di posisi target
        x = target_x;
        y = target_y;
        z = target_z;
        % Kecepatan sudah nol di akhir transisi berkat smoothstep
        xdot = 0;
        ydot = 0;
        zdot = 0;
    end
    
    % --- Perubahan Payload ---
    % Skenario perubahan payload tetap pada waktu yang Anda inginkan
    % Misalkan perubahan payload terjadi pada t = 5.0 detik seperti yang Anda definisikan di main_payload_UKF_NMPC.m
    % Trajectory referensi tidak secara langsung mengubah massa, melainkan NMPC akan bereaksi terhadap estimasi massa UKF.
    % Pastikan time_of_payload_change di main_payload_UKF_NMPC.m lebih besar dari end_transition_time
    % agar NMPC sempat stabil di posisi target sebelum massa berubah.

    % Orientasi (nol)
    phi = 0;
    theta = 0;
    psi = 0;

    % Kecepatan Sudut (nol)
    phidot = 0;
    thetadot = 0;
    psidot = 0;

    % Gabungkan semua state yang diinginkan
    xdesired = [x;y;z;phi;theta;psi;xdot;ydot;zdot;phidot;thetadot;psidot];
end