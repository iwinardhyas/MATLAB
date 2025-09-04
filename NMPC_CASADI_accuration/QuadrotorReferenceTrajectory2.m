function [ xdesired ] = QuadrotorReferenceTrajectory2( t )
    % Trajectory: Drone bergerak linear dari posisi awal ke posisi target
    % dengan kecepatan konstan.
    % Ini bagus untuk menguji kemampuan pelacakan kecepatan konstan.

    % --- Parameter Ramp ---
    initial_x = 0;
    initial_y = 0;
    initial_z = 0.5; 
    
    target_x = 2.0;
    target_y = -1.0;
    target_z = 3.0;

    duration = 5.0; % Durasi (detik) untuk mencapai target

    % Hitung kecepatan yang diperlukan untuk mencapai target dalam 'duration'
    vx_ref = (target_x - initial_x) / duration;
    vy_ref = (target_y - initial_y) / duration;
    vz_ref = (target_z - initial_z) / duration;

    % Posisi
    if t <= duration
        x = initial_x + vx_ref * t;
        y = initial_y + vy_ref * t;
        z = initial_z + vz_ref * t;
    else % Setelah durasi, tetap di posisi target
        x = target_x;
        y = target_y;
        z = target_z;
    end
    
    % Orientasi (nol)
    phi = 0;
    theta = 0;
    psi = 0;

    % Kecepatan (konstan selama ramp, lalu nol)
    if t <= duration
        xdot = vx_ref;
        ydot = vy_ref;
        zdot = vz_ref;
    else
        xdot = 0;
        ydot = 0;
        zdot = 0;
    end
    
    phidot = 0;
    thetadot = 0;
    psidot = 0;

    xdesired = [x;y;z;phi;theta;psi;xdot;ydot;zdot;phidot;thetadot;psidot];
end