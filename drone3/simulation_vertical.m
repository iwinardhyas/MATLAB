function sim_data = simulation_vertical(sys, mpc_ctrl, time, Ts, x, r)
    
    % Waktu Simulasi
    Tfinal = time; % 10 detik
    t = 0:Ts:Tfinal;
    
    % Inisialisasi MPCSTATE
    state = mpcstate(mpc_ctrl);    % Membuat objek state untuk MPC
    
    % Simulasi
    y = zeros(length(t), size(sys.C, 1)); % Output
    
    for k = 1:length(t)
        intermediate_pos = [0; 0; 5; 0; 0; 0];
        if norm(x(1:2) - intermediate_pos(1:2)) > 0.02 && abs(x(3) - intermediate_pos(3)) > 0.01
            u = mpcmove(mpc_ctrl, state, x, intermediate_pos); % Jika belum berada di posisi 0,0,5   
            fprintf('a1 = %f, a2 = %f\n',x(1:2) - intermediate_pos(1:2),abs(x(3) - intermediate_pos(3)))
        else
            u = mpcmove(mpc_ctrl, state, x, r); % Turun perlahan menuju 0,0,0
            disp('dasar')
        end

    % Update state sistem
    x = sys.A * x + sys.B * u;
    y(k, :) = sys.C * x + sys.D * u;
    
    % Periksa apakah sudah sangat dekat dengan target r
    if norm(x(1:3) - r(1:3)) < 0.01
        y = y(1:k, :);
        t = t(1:k);
        break;
    end
    end

    % Simpan hasil simulasi
    sim_data.Time = t';
    sim_data.Output = y;
end

