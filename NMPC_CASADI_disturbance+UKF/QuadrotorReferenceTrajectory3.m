function [ xdesired ] = QuadrotorReferenceTrajectory3( t )
    % Trajectory: Drone melakukan step change (lompatan) ke posisi baru
    % pada waktu tertentu.
    % Ini bagus untuk menguji overshoot, settling time, dan respons terhadap gangguan.

    % --- Parameter Step ---
    initial_x = 0;
    initial_y = 0;
    initial_z = 0.5; % Ketinggian awal
    
    target_x = 1.0;   % Posisi X target setelah step
    target_y = 1.0;   % Posisi Y target setelah step
    target_z = 2.0;   % Posisi Z target setelah step

    step_time = 2.0; % Waktu (detik) ketika step change terjadi

    % Posisi
    if t < step_time
        x = initial_x;
        y = initial_y;
        z = initial_z;
    else
        x = target_x;
        y = target_y;
        z = target_z;
    end
    
    % Orientasi (nol)
    phi = 0;
    theta = 0;
    psi = 0;

    % Kecepatan (nol, karena ini adalah step change instan)
    % NMPC akan secara implisit menghasilkan kecepatan untuk mencapai target.
    % Memberi kecepatan non-nol untuk step instan tidak masuk akal.
    xdot = 0;
    ydot = 0;
    zdot = 0;
    phidot = 0;
    thetadot = 0;
    psidot = 0;

    xdesired = [x;y;z;phi;theta;psi;xdot;ydot;zdot;phidot;thetadot;psidot];
end