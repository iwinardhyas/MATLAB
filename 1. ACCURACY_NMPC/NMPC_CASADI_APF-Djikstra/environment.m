function env = environment()
    % Lingkungan simulasi quadrotor dengan obstacle
    
    % Jumlah obstacle
    num_obs = 12;          
    obs_radius_val = 0.8;  % semua radius sama
    z_pos = 1.0;           % posisi dasar (anggap obstacle dari ground)
    z_height = 8.0;        % tinggi silinder
    
    % Pusat obstacle (manual supaya ada jarak cukup)
    obs_center = [ 6   6   8   12  15  18  22  24  26  29  32  36 ;   % x
                   9   3   1.8 4   6   2.8  2   8   4   6.8  12  16 ; % y
                   z_pos*ones(1,num_obs) ]; % z
    
    % Masukkan ke struct environment
    env.obstacles.center = obs_center;
    env.obstacles.radius = obs_radius_val * ones(1,num_obs);
    env.obstacles.z_min  = z_pos;
    env.obstacles.z_max  = z_height;
    
    % Tambahkan parameter peta (untuk Dijkstra)
    env.map.size   = [50, 35];  % m
    env.map.res    = 0.5;       % m/cell
    
    % Drone safety margin
    env.r_drone = 0.3;
    env.r_safe  = 0.2;
end
