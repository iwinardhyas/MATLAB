%% Simulasi Drone dengan MPC di MATLAB
% Definisi matriks model drone
A = [0 1 0 0 0 0;
     0 0 0 0 0 0;
     0 0 0 1 0 0;
     0 0 0 0 0 0;
     0 0 0 0 0 1;
     0 0 0 0 0 0];
 
B = [0 0 0;
     1 0 0;
     0 0 0;
     0 1 0;
     0 0 0;
     0 0 1];
 
C = eye(6);
D = zeros(6,3);

% Parameter MPC
Ts = 0.1;            % Waktu sampling
predictionHorizon = 10; % Horizon prediksi
controlHorizon = 3;     % Horizon kontrol

% Desain kontroler MPC
mpcController = mpc(ss(A, B, C, D, Ts), Ts);
mpcController.PredictionHorizon = predictionHorizon;
mpcController.ControlHorizon = controlHorizon;
mpcController.Weights.ManipulatedVariablesRate = [0.1 0.1 0.1];
mpcController.Weights.OutputVariables = [1 1 1 0.1 0.1 0.1];

% Batasan input
mpcController.MV = struct('Min', {-10, -10, -10}, 'Max', {10, 10, 10});

% Simulasi
Tsim = 20;  % Waktu simulasi dalam detik
t = 0:Ts:Tsim;
x = zeros(6, length(t));  % State awal
u = zeros(3, length(t)); % Input kontrol
x(:,1) = [0; 0; 0; 0; 0; 0]; % Inisialisasi state awal

for k = 1:length(t)-1
    % Referensi trajectory
    ref = QuadrotorReferenceTrajectory(t(k));
    yref = ref(1:6, k); % Ambil posisi dan orientasi target

    % Setpoint untuk MPC
    mpcState.Reference = yref;

    % Hitung input kontrol menggunakan MPC
    u(:, k) = mpcmove(mpcController, mpcState, x(:, k));

    % Update state menggunakan model linier
    x(:, k+1) = A * x(:, k) + B * u(:, k);
end

% % Plot hasil simulasi
% figure;
% subplot(3,1,1);
% plot(t, x(1,:), 'r', t, x(2,:), 'g', t, x(3,:), 'b');
% hold on;
% plot(t, QuadrotorReferenceTrajectory(t)(1,:), '--r', t, QuadrotorReferenceTrajectory(t)(2,:), '--g', t, QuadrotorReferenceTrajectory(t)(3,:), '--b');
% legend('x', 'y', 'z', 'x ref', 'y ref', 'z ref');
% xlabel('Time (s)'); ylabel('Position (m)'); title('Position Tracking');
% 
% subplot(3,1,2);
% plot(t, x(4,:), 'r', t, x(5,:), 'g', t, x(6,:), 'b');
% legend('\phi', '\theta', '\psi');
% xlabel('Time (s)'); ylabel('Orientation (rad)'); title('Orientation');
% 
% subplot(3,1,3);
% plot(t(1:end-1), u(1,:), 'r', t(1:end-1), u(2,:), 'g', t(1:end-1), u(3,:), 'b');
% legend('u_x', 'u_y', 'u_z');
% xlabel('Time (s)'); ylabel('Control Input'); title('Control Inputs');
