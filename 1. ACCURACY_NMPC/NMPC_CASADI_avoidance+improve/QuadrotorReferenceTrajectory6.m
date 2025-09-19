function [ xdesired ] = QuadrotorReferenceTrajectory6(t)
    trajectory_type = 'lurus';
    
    xdesired = zeros(12, 1);
    
    switch trajectory_type
        case 'step_input_test'
            amplitude_x = 2.0;
            amplitude_y = 2.0;
            forward_speed = 1.0;
            altitude = 1.5;

            % Posisi
            x = forward_speed * t;
            
            % Kondisi langkah (step) pada 2 detik
            if t < 2
                y = 0;
            else
                y = amplitude_y;
            end
            
            z = altitude;

            % Kecepatan
            xdot = forward_speed;
            if t < 2
                ydot = 0;
            else
                ydot = 0; % Kecepatan y akan berubah secara instan pada 2s
            end
            zdot = 0;

            % Perhitungan sudut dan kecepatan angular
            g = 9.81;
            phi = 0;
            theta = 0;
            psi = 0;

            phidot = 0;
            thetadot = 0;
            psidot = 0;

            xdesired = [x; y; z; phi; theta; psi; xdot; ydot; zdot; phidot; thetadot; psidot];
            
            case 'aggressive_attitude'
            radius = 2.0;       % Radius lingkaran
            angular_speed = 1.0; % Kecepatan angular, lebih cepat dari contoh Anda
            altitude = 2.0;
            g = 9.81;

            x = radius * cos(angular_speed * t);
            y = radius * sin(angular_speed * t);
            z = altitude;

            xdot = -radius * angular_speed * sin(angular_speed * t);
            ydot = radius * angular_speed * cos(angular_speed * t);
            zdot = 0;

            % Percepatan sentripetal
            a_centripetal = radius * angular_speed^2;

            % Perhitungan phi yang lebih agresif
            % phi = atan(a_centripetal / g)
            % phi akan meningkat seiring kecepatan angular
            phi = atan(a_centripetal / g);
            theta = 0;
            psi = 0;

            phidot = 0; 
            thetadot = 0; 
            psidot = 0; 

            xdesired = [x; y; z; phi; theta; psi; xdot; ydot; zdot; phidot; thetadot; psidot];
            
            case 'combined_test'
            amplitude = 1.0;
            frequency = 0.5;
            forward_speed = 0.5;
            altitude_step = 1.5;

            % Posisi: Gabungan sinus dan step
            x = forward_speed * t;
            y = amplitude * sin(2 * pi * frequency * t)+2;
            if t < 3
                z = 1.0;
            else
                z = altitude_step;
            end

            % Kecepatan
            omega = 2 * pi * frequency;
            xdot = forward_speed;
            ydot = amplitude * omega * cos(omega * t);
            
            if t < 3
                zdot = 0;
            else
                zdot = 0;
            end

            % Perhitungan sudut dan kecepatan angular
            g = 9.81;
            
            % Solusi: Gunakan turunan analitik untuk percepatan y
            ay = -amplitude * omega^2 * sin(omega * t); 
            ax = 0;
            
            phi = atan2(ay, g);
            theta = 0; 
            psi = 0; 

            phidot = 0; 
            thetadot = 0; 
            psidot = 0; 

            xdesired = [x; y; z; phi; theta; psi; xdot; ydot; zdot; phidot; thetadot; psidot];
            
        case 'lurus'
        % === PARAMETER TRAJEKTORI ===
        p_start = [0; 5; 0];
        p_goal = [50; 6.5; 5];
        T_total = 10; % Waktu total untuk mencapai tujuan (dalam detik)

        % === POSISI ===
        if t < T_total
            alpha = t / T_total;
            pos = (1 - alpha) * p_start + alpha * p_goal;
        else
            % Jika waktu sudah melewati T_total, drone tetap di posisi tujuan
            pos = p_goal;
        end

        % === KECEPATAN ===
        if t < T_total
            % Kecepatan konstan selama perjalanan
            vel = (p_goal - p_start) / T_total;
        else
            % Jika sudah mencapai tujuan, kecepatan menjadi nol
            vel = [0; 0; 0];
        end

        % === ORIENTASI ===
        % Roll (phi) dan Pitch (theta) diasumsikan nol untuk jalur datar
        phi = 0;
        theta = 0;
        % Yaw (psi) menghadap ke arah kecepatan
        yaw = atan2(vel(2), vel(1));

        % === KECEPATAN SUDUT ===
        % Kecepatan sudut diasumsikan nol untuk jalur lurus
        phidot = 0;
        thetadot = 0;
        psidot = 0;

        % === OUTPUT 12 STATE ===
        % x, y, z, phi, theta, psi, xdot, ydot, zdot, phidot, thetadot, psidot
        xdesired = [pos; phi; theta; yaw; vel; phidot; thetadot; psidot];
    end
end

function pos = QuadrotorReferenceTrajectory6_pos(t, p_start, p_waypt, p_goal, T1, T2)
    if t < T1
        alpha = t/T1;
        pos = (1-alpha)*p_start + alpha*p_waypt;
    elseif t < T2
        alpha = (t-T1)/(T2-T1);
        pos = (1-alpha)*p_waypt + alpha*p_goal;
    else
        pos = p_goal;
    end
end