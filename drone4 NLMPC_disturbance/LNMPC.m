import casadi.*

nx = 12;
ny = 12;
nu = 4;
nlmpcobj = nlmpc(nx, ny, nu);

nlmpcobj.Model.StateFcn = "QuadrotorStateFcn";

nlmpcobj.Jacobian.StateFcn = @QuadrotorStateJacobianFcn;

rng(0)

validateFcns(nlmpcobj,rand(nx,1),rand(nu,1));

Ts = 0.1;
p = 18;
m = 2;
nlmpcobj.Ts = Ts;
nlmpcobj.PredictionHorizon = p;
nlmpcobj.ControlHorizon = m;

nlmpcobj.MV = struct( ...
    Min={-15;-15;-15;-15}, ...
    Max={15;15;15;15}, ...
    RateMin={-3;-3;-3;-3}, ...
    RateMax={3;3;3;3} ...
    );

nlmpcobj.Weights.OutputVariables = [1 1 1 1 1 1 0 0 0 0 0 0];
nlmpcobj.Weights.ManipulatedVariables = [0.1 0.1 0.1 0.1];
nlmpcobj.Weights.ManipulatedVariablesRate = [0.1 0.1 0.1 0.1];

% Specify the initial conditions
x = [7;-10;0;0;0;0;0;0;0;0;0;0];

% Nominal control target (average to keep quadrotor floating)
nlopts = nlmpcmoveopt;
nlopts.MVTarget = [4.9 4.9 4.9 4.9]; 
mv = nlopts.MVTarget;

% Simulation duration in seconds
Duration = 20;

% Display waitbar to show simulation progress
hbar = waitbar(0,"Simulation Progress");

% MV last value is part of the controller state
lastMV = mv;


% Store states for plotting purposes
xHistory = x';
uHistory = lastMV;
gHistory = 1;

rho = 1.225; % Densitas udara (kg/m^3)
Cd = 0.2; % Koefisien drag (asumsi umum)
A_horizontal = 0.031; % Area proyeksi horizontal (m^2)
A_vertical = 0.00885; % Area proyeksi vertikal (m^2)
m = 2;

% standar, sedang,berat
% v=4-6,8-12,12-15 m/s
% a=2-3,3-5,5-7 m/s
% f = 0.2-0.5,1-2,1-2 hz

% Kecepatan angin dinamis
v_mean = 12; % Kecepatan angin rata-rata (m/s)
A_fluctuation = 3; % Amplitudo fluktuasi angin
f_wind = 2; % Frekuensi fluktuasi angin (Hz)

% Inisialisasi array untuk menyimpan noise
FxHistory = zeros(Duration/Ts, 1);
FyHistory = zeros(Duration/Ts, 1);
FzHistory = zeros(Duration/Ts, 1);

% Kegagalan aktuator pada detik ke-10
failure_time = 10;
failure_step = failure_time / Ts;
actuatorStatus = zeros(Duration/Ts + 1, 4);

% Simulation loop
for k = 1:(Duration/Ts)

    % Set references for previewing
    t = linspace(k*Ts, (k+p-1)*Ts,p);
    yref = QuadrotorReferenceTrajectory(t);

    % Compute control move with reference previewing
    xk = xHistory(k,:);
%     xk = x_est';
    
    %update hight (z) for ground effect
    z = xk(3);
    if z > 0 && z <0.25
        groundEffectFactor = 1 + (0.25/(16*z));
    else
        groundEffectFactor = 1;
    end
    
    disp('Nilai xk sebelum nlmpcmove:');
    disp(xk);
    if any(isnan(xk)) || any(isinf(xk))
        error('Nilai xk mengandung NaN atau Inf.');
    end
    
    try
        [uk, nlopts, info] = nlmpcmove(nlmpcobj, xk, lastMV, yref', [], nlopts);
    catch ME
        error('Gagal menghitung input kontrol: %s', ME.message);
    end
    
    disp('Info dari nlmpcmove:');
    disp(info);


    % Simulasi kegagalan aktuator
    if k >= failure_step
        %uk(2) = 0; % Motor 2 mati total
        uk(3) = 0.8 * uk(3); % Motor 3 hanya 50% efisiensi
    end
    actuatorStatus(k+1,:) = uk;
    % Store control move
%     uk = uk * groundEffectFactor;
    uHistory(k+1,:) = uk';
    lastMV = uk;
    
    
    v_wind_x = v_mean + A_fluctuation * sin(2 * pi * f_wind * t) + 0.5 * randn;
    v_wind_y = v_mean + A_fluctuation * cos(2 * pi * f_wind * t) + 0.5 * randn;
    v_wind_z = 0; % Angin biasanya horizontal
    
%   % Kecepatan relatif drone terhadap angin
    v_rel_x = xk(7) - v_wind_x; % xdot - v_wind_x
    v_rel_y = xk(8) - v_wind_y; % ydot - v_wind_y
    v_rel_z = xk(9) - v_wind_z; % zdot - v_wind_z
    
    F_drag_x = (0.5 * rho * Cd * A_horizontal * (v_rel_x.^2).* sign(v_rel_x)); % Drag arah x
    F_drag_y = (0.5 * rho * Cd * A_horizontal * (v_rel_y.^2).* sign(v_rel_y)); % Drag arah y
    F_drag_z = (0.5 * rho * Cd * A_vertical * (v_rel_z.^2).* sign(v_rel_z)); % Drag arah z
    
    % Batasi gaya drag agar tidak ekstrem
    F_drag_x = max(min(F_drag_x, 20), -20);
    F_drag_y = max(min(F_drag_y, 20), -20);
    F_drag_z = max(min(F_drag_z, 20), -20);

    
    % Simulate quadrotor for the next control interval (MVs = uk) 
    ODEFUN = @(t,xk) QuadrotorStateFcn(xk(:),uk) + [0;0;0;0;0;0;-F_drag_x(1);-F_drag_y(1);-F_drag_z(1); zeros(3, 1)];

    if any(isnan(xHistory(k,:))) || any(isinf(xHistory(k,:)))
        error('Nilai awal xHistory(k,:) tidak valid.');
    end
    if any(isnan(xk)) || any(isinf(xk))
        error('QuadrotorStateFcn: Nilai xk menjadi NaN atau Inf.');
    end
    
    opts = odeset('RelTol', 1e-5, 'AbsTol', 1e-7);
    [TOUT, XOUT] = ode45(ODEFUN, [0 Ts], xHistory(k,:)',opts);
%     try
%         options = odeset('RelTol', 1e-6, 'AbsTol', 1e-8);
%         [TOUT, XOUT] = ode15s(ODEFUN, [0 Ts], xHistory(k,:)', options);
%     catch ME
%         error('Gagal integrasi ODE: %s', ME.message);
%     end
    
    if any(isnan(XOUT(:))) || any(isinf(XOUT(:)))
        error('ODE solver menghasilkan NaN atau Inf!');
    end

    % Tambahan validasi hasil
    if any(isnan(XOUT(end,:))) || any(isinf(XOUT(end,:)))
        warning('ODE solver menghasilkan NaN/Inf pada step %d. Gunakan state sebelumnya.', k);
        XOUT(end,:) = xHistory(k,:); % fallback agar tidak error
    end


    % Update quadrotor state
    xHistory(k+1,:) = XOUT(end,:);
    gHistory = groundEffectFactor;
    FxHistory(k+1) = F_drag_x(1);
    FyHistory(k+1) = F_drag_y(1);
    FzHistory(k+1) = F_drag_z(1);
    
    % Update waitbar
    waitbar(k*Ts/Duration,hbar, sprintf('Progress: %.2f%%', (k*Ts/Duration)*100));
     % Cek jika proses selesai
    if k == (Duration/Ts)
        close(hbar);  % Tutup waitbar setelah selesai
    end
end

plotQuadrotorTrajectory;


