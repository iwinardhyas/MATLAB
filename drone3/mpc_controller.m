function mpc_ctrl = mpc_controller(sys, Ts)
    % MPC_CONTROLLER.M - Desain MPC Controller
    
    % Pastikan model adalah diskret
    if sys.Ts == 0
        error('Model harus dalam bentuk diskret. Gunakan c2d untuk mendiskretkan model.');
    end
    
    % Prediksi dan kontrol horizon
    prediction_horizon = 20;
    control_horizon = 5;
    
    % Batasan input dan output
    u_min = [-10; -10; -10]; u_max = [10; 10; 10];
%     y_min = [-1.45; -1.45; -5; -1.45; -1.45; -1.45]; % Batasan output minimum
%     y_max = [1.45; 1.45; 5; 1.45; 1.45; 1.45];     % Batasan output maksimum
    
    % Membuat objek MPC
     mpc_ctrl = mpc(sys, Ts, prediction_horizon, control_horizon);
    
    % Horizon MPC
    mpc_ctrl.PredictionHorizon = 30;
    mpc_ctrl.ControlHorizon = 5;
    
%     % Batasan Manipulated Variable (MV)
%     mpc_ctrl.MV.Min = u_min;
%     mpc_ctrl.MV.Max = u_max;

       % Tetapkan batasan input kontrol
    for i = 1:length(mpc_ctrl.MV)
        mpc_ctrl.MV(i).Min = u_min(i);
        mpc_ctrl.MV(i).Max = u_max(i);
    end
    
%     % Batasan Output Variable (OV)
%     for i = 1:length(mpc_ctrl.OV)
%         mpc_ctrl.OV(i).Min = y_min(i); % Atur batas minimum untuk setiap output
%         mpc_ctrl.OV(i).Max = y_max(i); % Atur batas maksimum untuk setiap output
%     end
    
    % Bobot
    mpc_ctrl.Weights.ManipulatedVariables = [0.01 0.01 0.01];      % Bobot untuk MV
    mpc_ctrl.Weights.ManipulatedVariablesRate = [0.1 0.1 0.1];   % Bobot untuk perubahan MV
    mpc_ctrl.Weights.OutputVariables = [1 1 1 1 1 1];      % Bobot untuk setiap output
end
