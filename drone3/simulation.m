function sim_data = simulation(sys, mpc_ctrl, time, Ts, x, r)
    % SIMULATION.M - Simulasi sistem
    
    % Waktu Simulasi
    Tfinal = time; % 10 detik
    t = 0:Ts:Tfinal;
    
    % Inisialisasi variabel
%     x = zeros(size(sys.A, 1), 1);   % State awal 
    
    % Inisialisasi MPCSTATE
    state = mpcstate(mpc_ctrl);    % Membuat objek state untuk MPC
    
    % Simulasi
    y = zeros(length(t), size(sys.C, 1)); % Output
    for k = 1:length(t)
        % Hitung kontrol MPC
        u = mpcmove(mpc_ctrl, state, x, r); % Gunakan state MPC
        
        % Update state sistem
        x = sys.A * x + sys.B * u;
        y(k, :) = sys.C * x + sys.D * u;
    end
    
    % Simpan hasil simulasi
    sim_data.Time = t';
    sim_data.Output = y;
end
