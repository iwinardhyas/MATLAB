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
    Min={0;0;0;0}, ... %batas thrust uk
    Max={10;10;10;10}, ...
    RateMin={-2;-2;-2;-2}, ... %batas thrust change
    RateMax={2;2;2;2} ...
    );

nlmpcobj.Weights.OutputVariables = [1 1 1 1 1 1 0 0 0 0 0 0]; %^matrix Q
nlmpcobj.Weights.ManipulatedVariables = [0.1 0.1 0.1 0.1]; % matrix R
nlmpcobj.Weights.ManipulatedVariablesRate = [0.1 0.1 0.1 0.1];

% Specify the initial conditions
x = [7;-10;0;0;0;0;0;0;0;0;0;0];

% Nominal control target (average to keep quadrotor floating)
nlopts = nlmpcmoveopt;
nlopts.MVTarget = [4.9 4.9 4.9 4.9]; %uk control
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

% Kecepatan angin dinamis
v_mean = 12; % Kecepatan angin rata-rata (m/s)
A_fluctuation = 5; % Amplitudo fluktuasi angin
f_wind = 2; % Frekuensi fluktuasi angin (Hz)

% Inisialisasi array untuk menyimpan noise
FxHistory = zeros(Duration/Ts, 1);
FyHistory = zeros(Duration/Ts, 1);
FzHistory = zeros(Duration/Ts, 1);

% Simulation loop
for k = 1:(Duration/Ts)

    % Set references for previewing
    t = linspace(k*Ts, (k+p-1)*Ts,p);
    yref = QuadrotorReferenceTrajectory(t);

    % Compute control move with reference previewing
    xk = xHistory(k,:);
    
    %update hight (z) for ground effect
    z = xk(3);
    if z > 0 && z <0.25
        groundEffectFactor = 1 + (0.25/(16*z));
    else
        groundEffectFactor = 1;
    end
    
    [uk,nlopts,info] = nlmpcmove(nlmpcobj,xk,lastMV,yref',[],nlopts);

    % Store control move
    uHistory(k+1,:) = uk';
    lastMV = uk / groundEffectFactor;
    
    % Generate dynamic Gaussian noise
%     mean_noise = 0;
%     std_noise = 0.9;
%     Fx_noise = mean_noise + std_noise * randn;
%     Fy_noise = mean_noise + std_noise * randn;
%     Fz_noise = mean_noise + std_noise * randn;
    
    v_wind_x = v_mean + A_fluctuation * sin(2 * pi * f_wind * t) + 0.5 * randn;
    v_wind_y = v_mean + A_fluctuation * cos(2 * pi * f_wind * t) + 0.5 * randn;
    v_wind_z = 0; % Angin biasanya horizontal
    
    F_drag_x = 0.5 * rho * Cd * A_horizontal * (v_wind_x.^2); % Drag arah x
    F_drag_y = 0.5 * rho * Cd * A_horizontal * (v_wind_y.^2); % Drag arah y
    F_drag_z = 0.5 * rho * Cd * A_vertical * (v_wind_z.^2); % Drag arah z
    
%     disp(size(F_drag_x(1)));

    % Simulate quadrotor for the next control interval (MVs = uk) 
%     ODEFUN = @(t,xk) QuadrotorStateFcn(xk,uk)+ [F_drag_x(1); F_drag_y(1); F_drag_z(1); zeros(9, 1)];
    ODEFUN = @(t,xk) QuadrotorStateFcn(xk,uk);
%     options = odeset('RelTol', 1e-6, 'AbsTol', 1e-8);
%     ODEFUN = @(t,xk) QuadrotorStateFcn(xk,uk);
    [TOUT,XOUT] = ode45(ODEFUN,[0 Ts], xHistory(k,:)');


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