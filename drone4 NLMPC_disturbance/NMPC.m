% Model linear
A = [0 0 0 1 0 0; 
     0 0 0 0 1 0; 
     0 0 0 0 0 1; 
     0 0 0 -0.1 0 0;
     0 0 0 0 -0.1 0;
     0 0 0 0 0 -0.1];  % Matriks A
B = [0 0 0;
    0 0 0;
    0 0 0;
    1 0 0;
    0 1 0;
    0 0 1];                % Matriks B (input)
C = eye(6); % Output all states
D = 0;                 % Matriks D

Ts = 0.1;
p = 1;
m = 2;

sys = ss(A, B, C, D);            % Sistem state-space kontinu
    
    % Diskretkan sistem jika diperlukan
sys = c2d(sys, Ts);

% Linear MPC
mpcobj = mpc(ss(A, B, C, D), Ts);
mpcobj.PredictionHorizon = p;
mpcobj.ControlHorizon = m;

u_min = [-10; -10; -10]; u_max = [10; 10; 10];
% Konfigurasi batasan kontrol
for i = 1:length(mpcobj.MV)
    mpc_ctrl.MV(i).Min = u_min(i);
    mpc_ctrl.MV(i).Max = u_max(i);
end

% Bobot
mpcobj.Weights.OutputVariables = [1 1 1 1 1 1];
mpcobj.Weights.ManipulatedVariables = [0.1 0.1 0.1];
mpcobj.Weights.ManipulatedVariablesRate = [0.1 0.1 0.1];

% Kondisi awal
x = [5;5;5;0;0;0];
uHistory(k,:) = [4.9 4.9 4.9 4.9]; % Kontrol awal
Duration = 20;
t = 0:Ts:Duration;
% disp(t);
state = mpcstate(mpcobj);

y = zeros(length(t), size(sys.C, 1)); % Output
% Inisialisasi
xHistory = zeros(Duration/Ts + 1, 6);  % Inisialisasi history untuk menyimpan semua state
xHistory(1, :)= x;  % State awal

% Simulasi loop
for k = 1:(Duration/Ts)
    % Set referensi untuk prediksi
    t = linspace(k*Ts, (k+p-1)*Ts,p);
    yref = QuadrotorReferenceTrajectory(t);
    
    % Hitung kontrol move
     u = mpcmove(mpcobj, state,x,yref);
     
%     ODEFUN = @(t, x) sys.A * x + sys.B * u;
%     y(k, :) = sys.C * x + sys.D * u;
%     [TOUT, XOUT] = ode45(ODEFUN, [0 Ts], x');
%     xHistory(k+1,:) = XOUT(end,:);
%     disp(xHistory);
    [t, x] = ode45(@(t, x) linearMPCODE(t, x, A, B, u), k, x);


%     x = (sys.A * x + sys.B * u);
%     y(k, :) = sys.C * x + sys.D * u;
%     disp(x');

end